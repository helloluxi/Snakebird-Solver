using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Text.Json;
using int2 = System.ValueTuple<int, int>;
using ints = System.Collections.Generic.List<int>;
using int2s = System.Collections.Generic.List<System.ValueTuple<int, int>>;

#if UseBigInt
using number = System.UInt128;
#else
using number = System.UInt64;
#endif

public struct Hueristic{
    public bool useNonSymDistance { get; set; }
    public int stepWeight { get; set; }
    public int birdToTargWeight { get; set; }
    public int fruitMinDistToBirdWeight { get; set; }
    public int frameToHeuTargWeight { get; set; }
    public int birdFollowingFrameWeight { get; set; }
    public int extraPreparingCost { get; set; }
    public ints frameTargets { get; set; }
    public int2s realFrameTargets;
}

public class LevelJson {
    // The first 2 ints define the head position, the followings define directions
    // 0: Right, 1: Left, 2: Up, 3:Down
    public List<ints> birds{ get; set; }
    // The first 2 ints define the center position, the followings define relative positions
    public List<ints> frames{ get; set; }
    public ints fruits{ get; set; }
    public ints spikes{ get; set; }
    public ints walls{ get; set; } // Magic number 9999 defines a rectangle

    public int[] target{ get; set; } // Length = 2
    public int[] range{ get; set; } // Length = 4
    public int[] portals{ get; set; } // Length = 4 or 0
    public Hueristic hueristic { get; set; }
    public int shapeKeyLength { get; set; }
}

public class Level {
    public const int EMPTY = 0, WALL = 1, SPIKE = 2, FRUIT_BASE = 32, OBJECT_BASE = 64;
    public int[,] initMap;
    public List<ints> birdShapes;
    public List<int2s> frameShapes;
    public int2s fruitPos, portalPos;
    public int2 target;
    public int width, height, posBit;
    public int birdCount;
    public int frameCount => frameShapes.Count;
    public int fruitCount => fruitPos.Count;
    public int objectCount => frameCount + birdCount;
}

public class State {
    public class Bird { public int2 pos; public int shapeKey; }
    public List<Bird> birds;
    public int2s frames;
    public uint fruitFreshFlag;
    public int step;
}

public static class Tools{
    public static int2s ReduceBy2(this IList<int> flatArr, int rangeX = 0, int rangeY = 0){
        return Enumerable.Range(0, flatArr.Count / 2).Select(i => (flatArr[i*2] - rangeX, flatArr[i*2+1] - rangeY)).ToList();
    }
    public static int Get(this int[,] map, int2 pos) => map[pos.Item1, pos.Item2];
    public static void Set(this int[,] map, int2 pos, int value) => map[pos.Item1, pos.Item2] = value;
    public static int2 Add(this int2 a, int2 b) => (a.Item1 + b.Item1, a.Item2 + b.Item2);
    public static int2 Sub(this int2 a, int2 b) => (a.Item1 - b.Item1, a.Item2 - b.Item2);
    public static void WallReader(this int[,] map, int[] range, int written, List<int> description){
        int idx = -1;
        while(idx < description.Count - 1){
            if(description[++idx] == 9999){
                int xMin = description[++idx], xMax = description[++idx],
                    yMin = description[++idx], yMax = description[++idx];
                for(int i = xMin; i <= xMax; i++){
                    for(int j = yMin; j <= yMax; j++){
                        map[i - range[0], j - range[2]] = written;
                    }
                }
            }
            else{
                map[description[idx] - range[0], description[++idx] - range[2]] = written;
            }
        }
    }
}

class Program {
    static void Main(string[] args)
    {
        var levelReader = JsonSerializer.Deserialize<LevelJson>(File.ReadAllText("level.json"));

        // Read static level info
        var initMap = new int[levelReader.range[1] - levelReader.range[0] + 1, levelReader.range[3] - levelReader.range[2] + 1];
        var level = new Level{
            initMap = initMap,
            width = initMap.GetLength(0),
            height = initMap.GetLength(1),
            posBit = BitOperations.Log2((uint)initMap.Length) + 1,
            birdShapes = levelReader.birds.Select(bird => bird.Skip(2).ToList()).ToList(),
            frameShapes = levelReader.frames.Select(arr => { var newArr = arr.ReduceBy2(); newArr[0] = default; return newArr; }).ToList(),
            fruitPos = levelReader.fruits.ReduceBy2(levelReader.range[0], levelReader.range[2]),
            birdCount = levelReader.birds.Count,
            target = (levelReader.target[0] - levelReader.range[0], levelReader.target[1] - levelReader.range[2]),
        };
        level.initMap.WallReader(levelReader.range, Level.WALL, levelReader.walls);
        level.initMap.WallReader(levelReader.range, Level.SPIKE, levelReader.spikes);

        // Read the init state
        var initState = new State{
            birds = levelReader.birds.Select(ls => new State.Bird{ pos = (ls[0] - levelReader.range[0], ls[1] - levelReader.range[2]) }).ToList(),
            frames = levelReader.frames.Select(ls => (ls[0] - levelReader.range[0], ls[1] - levelReader.range[2])).ToList(),
            fruitFreshFlag = (1U << level.fruitCount) - 1
        };
        
        // Solve the level
        var solver = new Solver{
            level = level,
            hueristic = levelReader.hueristic
        };
        solver.hueristic.realFrameTargets = solver.hueristic.frameTargets.ReduceBy2(levelReader.range[0], levelReader.range[2]);
        if(levelReader.shapeKeyLength != 0)
            solver.shapeKeyLength = levelReader.shapeKeyLength;
#if !UseBigInt
        // Overflow check
        int bitUsed = (level.posBit + solver.shapeKeyLength) * level.birdCount + 
            solver.shapeKeyLength * level.frameCount + level.fruitCount;
        if (bitUsed + 4 >= sizeof(number) * 8) throw new Exception($"Too many bits used: {bitUsed}");
#endif
        solver.Solve(initState);
    }
}

public class Solver{
    public Level level;
    public Hueristic hueristic;
    public int shapeKeyLength = 6;
    private const string DIRECTIONS = "RLUD";
    private const int SHAPE_NOT_COMPUTED = -1, SHAPE_ILLEGAL = -2;
    private readonly List<int[]> birdShapeCache = new ();
    private readonly List<int[]> shapeKeyMap = new ();

    private number Serialize(State state){
        // Format: [BirdPos][FramePos][FruitFreshFlag]
        number s = (number)state.fruitFreshFlag;
        int idx = level.fruitCount;

        for(int frameObjIdx = 0; frameObjIdx < level.frameCount; frameObjIdx++){
            s |= (number)(state.frames[frameObjIdx].Item1 + level.width * state.frames[frameObjIdx].Item2) << idx;
            idx += level.posBit;
        }

        foreach(var bird in state.birds.Where(bird => bird.pos != NullPos).OrderBy(bird => bird.pos.Item1).ThenBy(bird => bird.pos.Item2)){
            // The first `posBit` bits are the position of the bird
            s |= (number)(bird.pos.Item1 + bird.pos.Item2 * level.width) << idx;
            idx += level.posBit;
            s |= (number)bird.shapeKey << idx;
            idx += shapeKeyLength;
        }
        return s;
    }
    

#region Heuristic
    private static int Distance(int2 a, int2 b){
        return Math.Abs(a.Item1 - b.Item1) + Math.Abs(a.Item2 - b.Item2);
    }
    private static int NonSymmetricDistance(int2 a, int2 b){
        return Math.Abs(a.Item1 - b.Item1) + 2 * Math.Max(b.Item2 - a.Item2, 0);
    }
    private int Hueristic(State state){
        Func<int2, int2, int> distFunc = hueristic.useNonSymDistance ? NonSymmetricDistance : Distance;
        int h = state.step * hueristic.stepWeight;

        int fruitCost = hueristic.fruitMinDistToBirdWeight == 0 ? 0 :
            Enumerable.Range(0, level.fruitCount)
                .Where(fruitIdx => (state.fruitFreshFlag & (1U << fruitIdx)) != 0)
                .Select(fruitIdx => state.birds
                    .Where(bird => bird.pos != NullPos)
                    .Select(bird => distFunc(bird.pos, level.fruitPos[fruitIdx]))
                    .Min())
                .Sum();
        h += fruitCost * hueristic.fruitMinDistToBirdWeight;
        
        int frameTargetCost = hueristic.frameTargets == null ? 0 :
            Enumerable.Range(0, level.frameCount)
                .Where(frameIdx => state.frames[frameIdx] != NullPos)
                .Select(frameIdx => distFunc(state.frames[frameIdx], hueristic.realFrameTargets[frameIdx]))
                .Sum();
        h += frameTargetCost * hueristic.frameToHeuTargWeight;

        // Ignore if all frames are in target
        int birdFollowingFrameCost = hueristic.birdFollowingFrameWeight == 0 || frameTargetCost == 0 ? 0 :
            state.birds.Where(bird => bird.pos != NullPos)
                .Sum(bird => state.frames
                    .Where(frame => frame != NullPos)
                    .Min(frame => distFunc(frame, bird.pos)));
        h += birdFollowingFrameCost * hueristic.birdFollowingFrameWeight;

        // Ignore if there is remaining fruits or frames
        int birdToTargCost = hueristic.extraPreparingCost != 0 && (fruitCost != 0 || frameTargetCost != 0) ? 
            hueristic.extraPreparingCost :
            state.birds.Where(bird => bird.pos != NullPos)
                .Sum(bird => distFunc(bird.pos, level.target)) * hueristic.birdToTargWeight;
        h += birdToTargCost;

        return h;
    }
#endregion

#region Mechanics
    private bool RecursiveTest(int[,] map, State state, int idx, int dir, bool[] moveList){
        if (moveList[idx]) return true;
        moveList[idx] = true;
        foreach(int2 pos in GetParts(state, idx)){
            int2 nextPos = Move(pos, dir);
            if (OutOfBound(nextPos)) return false;
            int obj = map[nextPos.Item1, nextPos.Item2];
            if(obj >= Level.OBJECT_BASE){
                if(!moveList[obj - Level.OBJECT_BASE] &&
                    IsValid(state, obj - Level.OBJECT_BASE) && 
                    !RecursiveTest(map, state, obj - Level.OBJECT_BASE, dir, moveList)){
                    return false;
                }
            }
            else if(obj != Level.EMPTY){
                return false;
            }
        }
        return true;
    }
    private bool TestSnakeMove(int[,] map, State state, int idx, int dir, bool[] moveList){
        var nextPos = Move(state.birds[idx].pos, dir);
        Array.Fill(moveList, false);
        if (map[nextPos.Item1, nextPos.Item2] >= Level.OBJECT_BASE){
            return RecursiveTest(map, state, map[nextPos.Item1, nextPos.Item2] - Level.OBJECT_BASE, dir, moveList) && !moveList[idx];
        }
        else
        { 
            return map[nextPos.Item1, nextPos.Item2] is >= Level.FRUIT_BASE or Level.EMPTY;
        }
    }
    private bool TestEntityMove(int[,] map, State state, int idx, int dir, bool[] moveList)
    {
        Array.Fill(moveList, false);
        moveList[idx] = true;
        foreach(int2 pos in GetParts(state, idx))
        {
            var nextPos = Move(pos, dir);
            if (!OutOfBound(nextPos) &&
                (map[nextPos.Item1, nextPos.Item2] >= Level.OBJECT_BASE ?
                !RecursiveTest(map, state, map[nextPos.Item1, nextPos.Item2] - Level.OBJECT_BASE, dir, moveList) :
                map[nextPos.Item1, nextPos.Item2] is >= Level.FRUIT_BASE or Level.WALL ||
                map[nextPos.Item1, nextPos.Item2] == Level.SPIKE && idx >= level.birdCount))
            {
                return false;
            }
        }
        return true;
    }
    private int FindShapeIndex(ints shape){
        for(int i = 0; i < birdShapeCache.Count; i++){
            if(Enumerable.SequenceEqual(birdShapeCache[i], shape)){
                return i;
            }
        }
        birdShapeCache.Add(shape.ToArray());
        if (birdShapeCache.Count > (1 << shapeKeyLength))
            throw new Exception("Shape Key Overflow");
        shapeKeyMap.Add(new int[level.fruitCount == 0 ? 4 : 8]);
        Array.Fill(shapeKeyMap.Last(), SHAPE_NOT_COMPUTED);
        shapeKeyMap.Last()[shape[0]] = SHAPE_ILLEGAL;
        return birdShapeCache.Count - 1;
    }
    private int NextShape(int shape, int dir, bool eatFruit){
        int nextShape = shapeKeyMap[shape][dir | (eatFruit ? 4 : 0)];
        if(nextShape != SHAPE_NOT_COMPUTED) return nextShape;
        // Calculate new shape
        ints shapeDetail = birdShapeCache[shape].ToList();
        if(!eatFruit)
            shapeDetail.RemoveAt(shapeDetail.Count - 1);
        shapeDetail.Insert(0, dir ^ 1);
        nextShape = FindShapeIndex(shapeDetail);
        return shapeKeyMap[shape][dir | (eatFruit ? 4 : 0)] = nextShape;
    }
    private bool ApplyMove(int[,] map, State state, int idx, int dir, bool snakeMove, bool[] moveList){
        if(snakeMove){
            // If the bird falls out, report illegal
            int2 nextPos = Move(state.birds[idx].pos, dir);
            if (OutOfBound(nextPos))
                return false;
            bool eatFruit = map.Get(nextPos) is >= Level.FRUIT_BASE and < Level.OBJECT_BASE;
            if(eatFruit)
                state.fruitFreshFlag &= ~(1U << (map.Get(nextPos) - Level.FRUIT_BASE));
            state.birds[idx].pos = nextPos;
            state.birds[idx].shapeKey = NextShape(state.birds[idx].shapeKey, dir, eatFruit);
            
            // Test Portal
            if (level.portalPos != null && level.portalPos.Contains(nextPos)){
                int portalIdx = level.portalPos.IndexOf(nextPos);
                int2 deltaPos = level.portalPos[portalIdx ^ 1].Sub(level.portalPos[portalIdx]);
                state.birds[idx].pos = state.birds[idx].pos.Add(deltaPos);
            }
            // If the bird reaches target, remove it
            if (state.fruitFreshFlag == 0 && state.birds[idx].pos == level.target)
                state.birds[idx].pos = NullPos;
        }
        for (int objIdx = 0; objIdx < level.objectCount; objIdx++){
            if(moveList[objIdx]){
                // Move to new pos
                if(objIdx < level.birdCount){
                    state.birds[objIdx].pos = Move(state.birds[objIdx].pos, dir);
                    // Check target
                    if (state.birds[objIdx].pos == level.target) {
                        state.birds[objIdx].pos = NullPos;
                        continue;
                    }
                }
                else{
                    state.frames[objIdx - level.birdCount] = Move(state.frames[objIdx - level.birdCount], dir);
                }

                int portalIdx = -1;
                var objParts = GetParts(state, objIdx);
                foreach(int2 pos in objParts){
                    // falls out
                    if(!snakeMove && pos.Item2 <= 0){
                        // If the bird or the needed frame falls out, report illegal
                        if (objIdx < level.birdCount || hueristic.frameTargets != null)
                            return false;
                        state.frames[objIdx - level.birdCount] = NullPos;
                        break;
                    }
                    // If the bird touches spike, report illegal
                    if(objIdx < level.birdCount &&
                        map[pos.Item1, pos.Item2] == Level.SPIKE)
                        return false;
                    // Test Portal
                    if(level.portalPos != null && portalIdx == -1 && level.portalPos.Contains(pos)){
                        portalIdx = level.portalPos[0] == pos ? 0 : 1;
                        if(objIdx < level.birdCount){
                            state.birds[objIdx].pos.Item1 += level.portalPos[portalIdx ^ 1].Item1 - level.portalPos[portalIdx].Item1;
                            state.birds[objIdx].pos.Item2 += level.portalPos[portalIdx ^ 1].Item2 - level.portalPos[portalIdx].Item2;
                        }
                        else{
                            state.frames[objIdx - level.birdCount] = (
                                state.frames[objIdx - level.birdCount].Item1 + level.portalPos[portalIdx ^ 1].Item1 - level.portalPos[portalIdx].Item1,
                                state.frames[objIdx - level.birdCount].Item2 + level.portalPos[portalIdx ^ 1].Item2 - level.portalPos[portalIdx].Item2
                            );
                        }
                    }
                }
            }
        }
        // Update map
        CopyStateToMap(state, map);
        return true;
    }
#endregion

    public void Solve(State initState){
        NullPos = (((1 << level.posBit) - 1, 0));
        for (int i = 0; i < initState.birds.Count; i++)
            initState.birds[i].shapeKey = FindShapeIndex(level.birdShapes[i]);
        var initStateSerial = Serialize(initState);

        int[,] map = new int[level.width, level.height];
        CopyStateToMap(initState, map);
        PrintMap(map);

        var route = new Dictionary<number, number>();
        var queue = new PriorityQueue<State, int>();
        queue.Enqueue(initState, Hueristic(initState));
        bool[] moveList = new bool[level.objectCount], activeList = new bool[level.objectCount];
        while(queue.Count > 0){
            var oldState = queue.Dequeue();
            var oldStateSerial = Serialize(oldState);
            for(int birdIdx = 0; birdIdx < level.birdCount; birdIdx++){
                if (oldState.birds[birdIdx].pos == NullPos) continue;
                for (int dir = 0; dir < 4; dir++){
                    int2 nextPos = Move(oldState.birds[birdIdx].pos, dir);

                    // Pre-test illegal move
                    if(OutOfBound(nextPos) ||
                        level.initMap[nextPos.Item1, nextPos.Item2] is Level.WALL or Level.SPIKE ||
                        shapeKeyMap[oldState.birds[birdIdx].shapeKey][dir] == SHAPE_ILLEGAL){
                        continue;
                    }
                    CopyStateToMap(oldState, map);
                    if (!TestSnakeMove(map, oldState, birdIdx, dir, moveList)){
                        continue;
                    }

                    // Push
                    var newState = new State{
                        birds = oldState.birds.Select(bird => new State.Bird{ pos = bird.pos, shapeKey = bird.shapeKey }).ToList(),
                        frames = oldState.frames.ToList(),
                        fruitFreshFlag = oldState.fruitFreshFlag,
                        step = oldState.step + 1,
                    };
                    if(!ApplyMove(map, newState, birdIdx, dir, true, moveList))
                        goto ILLEGAL_MOVE;

                    // Fall
                    Array.Fill(activeList, true);
                    while(true)
                    {
                        int fallIdx = Array.FindIndex(activeList, x => x);
                        if (fallIdx < 0)
                            break;
                        if (!IsValid(newState, fallIdx) || !TestEntityMove(map, newState, fallIdx, 3, moveList))
                        {
                            activeList[fallIdx] = false;
                            continue;
                        }
                        if(!ApplyMove(map, newState, birdIdx, 3, false, moveList))
                            goto ILLEGAL_MOVE;
                        Array.Fill(activeList, true);
                    }
                    
                    // Reduce memory
                    if(Enumerable.SequenceEqual(newState.frames, oldState.frames))
                        newState.frames = oldState.frames;

                    // Check if the level is solved
                    if (newState.birds.All(bird => bird.pos == NullPos)){
                        // print solution
                        Console.WriteLine();
                        var ls = new Stack<int2>();
                        ls.Push((birdIdx, dir));
                        number key = oldStateSerial;
                        while(initStateSerial != key){
                            number value = route[key];
                            ls.Push(((int)((value >> 2) & 3), (int)(value & 3)));
                            key = value >> 4;
                        }
                        Console.Write("Solution = ");
                        var sb = new StringBuilder();
                        birdIdx = -1;
                        while(ls.Count > 0){
                            var (solIdx, solDir) = ls.Pop();
                            if(birdIdx != solIdx){
                                birdIdx = solIdx;
                                sb.Append($" [{solIdx + 1}]");
                            }
                            sb.Append($"{DIRECTIONS[solDir]}");
                        }
                        Console.WriteLine(sb.ToString());
                        Console.WriteLine($"({route.Count} nodes searched)");
                        return;
                    }
                    else{
                        // save to queue
                        var newStateSerial = Serialize(newState);
                        if(route.TryAdd(newStateSerial, oldStateSerial << 4 | (number)(birdIdx << 2 | dir))) {
                            queue.Enqueue(newState, Hueristic(newState));
#if CheckIntermediate
                            if(route.Count % 100000 == 0){
                                PrintMap(map);
                                Console.WriteLine($"Searched={route.Count}\nPending={queue.Count}\nHeuristic={Hueristic(newState)}");
                            }
#endif
                        }
                    }

                ILLEGAL_MOVE:;
                }
            }
        }
    }

#region Tools
    private static int2 NullPos;
    private bool IsValid(State state, int objIdx) => (objIdx < level.birdCount ? state.birds[objIdx].pos :
        state.frames[objIdx - level.birdCount]) != NullPos;
    private IEnumerable<int2> GetParts(State state, int objIdx) => objIdx < level.birdCount ? GetBirdPos(state.birds[objIdx]) :
        GetFramePos(state.frames[objIdx - level.birdCount], level.frameShapes[objIdx - level.birdCount]);
    public static IEnumerable<int2> GetFramePos(int2 head, int2s shape){
        foreach (var pos in shape)
            yield return (head.Item1 + pos.Item1, head.Item2 + pos.Item2);
    }
    public IEnumerable<int2> GetBirdPos(State.Bird bird){
        int2 head = bird.pos;
        yield return head;
        foreach(var i in birdShapeCache[bird.shapeKey]){
            switch(i){
                case 0: head.Item1++; break;
                case 1: head.Item1--; break;
                case 2: head.Item2++; break;
                case 3: head.Item2--; break;
            }
            yield return head;
        }
    }
    private static int2 Move(int2 pos, int dir){
        return (pos.Item1 + (dir == 0 ? 1 : dir == 1 ? -1 : 0), pos.Item2 + (dir == 2 ? 1 : dir == 3 ? -1 : 0));
    }
    private bool OutOfBound(int2 pos){
        return pos.Item1 < 0 || pos.Item1 >= level.width || pos.Item2 < 0 || pos.Item2 >= level.height;
    }
    private void CopyStateToMap(State state, int[,] map){
        Array.Copy(level.initMap, map, level.initMap.Length);
        for(int objIdx = 0; objIdx < level.objectCount; objIdx++){
            if (IsValid(state, objIdx)){
                foreach (var pos in GetParts(state, objIdx)){
                    map[pos.Item1, pos.Item2] = Level.OBJECT_BASE | objIdx;
                }
            }
        }
        for(int fruitIdx = 0; fruitIdx < level.fruitCount; fruitIdx++){
            if(((state.fruitFreshFlag >> fruitIdx) & 1) == 1){
                map[level.fruitPos[fruitIdx].Item1, level.fruitPos[fruitIdx].Item2] = Level.FRUIT_BASE | fruitIdx;
            }
        }
    }
    private void PrintMap(int[,] map)
    {
        Console.WriteLine("\n");
        for (int i = level.height - 1; i >= 0; i--)
        {
            for (int j = 0; j < level.width; j++)
            {
                char c = map[j, i] switch
                {
                    Level.WALL => '#',
                    Level.SPIKE => 'X',
                    >= Level.OBJECT_BASE => (char)('1' + map[j, i] - Level.OBJECT_BASE),
                    >= Level.FRUIT_BASE and < Level.OBJECT_BASE => (char)('a' + map[j, i] - Level.FRUIT_BASE),
                    _ => '_'
                };
                if(level.target.Item1 == j && level.target.Item2 == i)
                    c = 'O';
                else if(c == '_' && level.portalPos != null && level.portalPos.Contains((j, i)))
                    c = '@';
                Console.Write(c);
                Console.Write(' ');
            }
            Console.WriteLine();
        }
    }
#endregion

}