using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;
using System.Text;
using System.Text.Json;
using System.Threading.Tasks;
using int2 = System.ValueTuple<int, int>;
using int2s = System.Collections.Generic.List<System.ValueTuple<int, int>>;

#if Use128Bit
using number = System.UInt128;
#else
using number = System.UInt64;
#endif

public class LevelReader {
    // The first 2 ints define the head position, the followings define directions
    // 0: Right, 1: Left, 2: Up, 3:Down
    public List<int[]> birds{ get; set; }
    // The first 2 ints define the center position, the followings define relative positions
    public List<int[]> frames{ get; set; }
    // Every 2 ints define a position
    public List<int> fruits{ get; set; }
    public List<int> spikes{ get; set; }
    public List<int> frameHeuristics { get; set; }
    public List<int> walls{ get; set; } // Magic number 9999 defines a rectangle

    public int[] target{ get; set; } // Length = 2
    public int[] range{ get; set; } // Length = 4
    public int[] portals{ get; set; } // Length = 4 or 0
    
    public int fruitMultiplier { get; set; }
    public int frameMultiplier { get; set; }
    public int stepMultiplier { get; set; }
    public int targetMultiplier { get; set; }
    public int birdFrameMultiplier { get; set; }
    public int allFruitAndFrameBonus { get; set; }
}

public class LevelStatic {
    public const int EMPTY = 0, WALL = 1, SPIKE = 2, FRUIT_BASE = 3, OBJECT_BASE = 30;
    public int[,] initMap;
    public int2[] fruitPos, portalPos;
    public int2 target;
    public int width, height, posBit;
    public int birdCount, frameCount;
    public int fruitCount => fruitPos.Length;
}

public class LevelState {
    public int2s[] objects;
    public uint fruitFresh;
    public int step;
    public number Serialize(LevelStatic level){
        number s = (number)fruitFresh;
        int idx = level.fruitCount;
        for(int frameObjIdx = level.birdCount; frameObjIdx < objects.Length; frameObjIdx++){
            if (objects[frameObjIdx] == null) {
                s |= (((number)1 << level.posBit) - 1) << idx;
            }
            else {
                int2 framePos = objects[frameObjIdx][0];
                s |= (number)(framePos.Item1 + framePos.Item2 * level.width) << idx;
            }
            idx += level.posBit;
        }
        for(int birdIdx = 0; birdIdx < level.birdCount; birdIdx++)
        {
            if (objects[birdIdx] == null)
            {
                s |= (((number)1 << level.posBit) - 1) << idx;
                idx += level.posBit;
            }
            else
            {
                // The first `posBit` bits are the position of the bird
                s |= (number)(objects[birdIdx][0].Item1 + objects[birdIdx][0].Item2 * level.width) << idx;
                idx += level.posBit;
                // Every 2 bits in the middle are the directions of the bird
                for (int dirIdx = 1; dirIdx < objects[birdIdx].Count; dirIdx++)
                {
                    int2 thisPos = objects[birdIdx][dirIdx], lastPos = objects[birdIdx][dirIdx - 1];
                    s |= (number)(lastPos.Item1 > thisPos.Item1 ? 0 :
                        lastPos.Item1 < thisPos.Item1 ? 1 :
                        lastPos.Item2 > thisPos.Item2 ? 2 : 3
                    ) << idx;
                    idx += 2;
                }
                // Inversed 2 bits denotes the end of the bird, assert the bird length is not 1
                s |= ((s >> (idx - 2)) ^ (number)1) << idx;
                idx += 2;
            }
        }
        return s;
    }
    public void Deserialize(LevelStatic level, LevelState initState, number key){
        step = -1;
        fruitFresh = (uint)key & ((1U << level.fruitCount) - 1);

        int idx = level.fruitCount;
        for(int frameObjIdx = level.birdCount; frameObjIdx < objects.Length; frameObjIdx++){
            int head = (int)(key >> idx) & ((1 << level.posBit) - 1);
            idx += level.posBit;
            if (head == (1 << level.posBit) - 1) {
                objects[frameObjIdx] = null;
            }
            else {
                objects[frameObjIdx] = initState.objects[frameObjIdx].Select(pos => (
                    pos.Item1 - initState.objects[frameObjIdx][0].Item1 + head % level.width,
                    pos.Item2 - initState.objects[frameObjIdx][0].Item2 + head / level.width)
                ).ToList();
            }
        }
        for(int birdIdx = 0; birdIdx < level.birdCount; birdIdx++)
        {
            int head = (int)(key >> idx) & ((1 << level.posBit) - 1), lastDir = -1;
            idx += level.posBit;
            if (head == (1 << level.posBit) - 1) {
                objects[birdIdx] = null;
            }
            else {
                objects[birdIdx] = new int2s{(
                    head % level.width,
                    head / level.width
                )};
                while (true)
                {
                    int dir = (int)((key >> idx) & 3);
                    idx += 2;
                    if (dir == (lastDir ^ 1)) break;
                    lastDir = dir;
                    objects[birdIdx].Add((
                        objects[birdIdx][^1].Item1 + (dir == 0 ? -1 : dir == 1 ? 1 : 0),
                        objects[birdIdx][^1].Item2 + (dir == 2 ? -1 : dir == 3 ? 1 : 0)
                    ));
                }
            }
        }
    }
}

class Program {
    static void Main(string[] args)
    {
        var levelReader = JsonSerializer.Deserialize<LevelReader>(File.ReadAllText("level.json"));

        // Read static level info
        var initMap = new int[levelReader.range[1] - levelReader.range[0] + 1, levelReader.range[3] - levelReader.range[2] + 1];
        var level = new LevelStatic{
            initMap = initMap,
            width = initMap.GetLength(0),
            height = initMap.GetLength(1),
            posBit = BitOperations.Log2((uint)initMap.Length) + 1, //(BitOperations.IsPow2(initMap.Length) ? 0 : 1),
            fruitPos = new int2[levelReader.fruits.Count / 2],
            birdCount = levelReader.birds.Count,
            frameCount = levelReader.frames.Count,
            target = (levelReader.target[0] - levelReader.range[0], levelReader.target[1] - levelReader.range[2]),
        };
        int wallReader = -1;
        while(wallReader < levelReader.walls.Count - 1){
            if(levelReader.walls[++wallReader] == 9999){
                int xMin = levelReader.walls[++wallReader], xMax = levelReader.walls[++wallReader],
                    yMin = levelReader.walls[++wallReader], yMax = levelReader.walls[++wallReader];
                for(int i = xMin; i <= xMax; i++){
                    for(int j = yMin; j <= yMax; j++){
                        level.initMap[i - levelReader.range[0], j - levelReader.range[2]] = LevelStatic.WALL;
                    }
                }
            }
            else{
                level.initMap[levelReader.walls[wallReader] - levelReader.range[0], levelReader.walls[++wallReader] - levelReader.range[2]] = LevelStatic.WALL;
            }
        }
        for (int i = 0; i < levelReader.spikes.Count / 2; i++)
        {
            level.initMap[levelReader.spikes[i*2] - levelReader.range[0], levelReader.spikes[i*2+1] - levelReader.range[2]] = LevelStatic.SPIKE;
        }
        for (int i = 0; i < level.fruitCount; i++)
        {
            level.fruitPos[i] = (levelReader.fruits[i*2] - levelReader.range[0], levelReader.fruits[i*2+1] - levelReader.range[2]);
        }
        if(levelReader.portals.Length == 4){
            level.portalPos = new int2[]{
                (levelReader.portals[0] - levelReader.range[0], levelReader.portals[1] - levelReader.range[2]),
                (levelReader.portals[2] - levelReader.range[0], levelReader.portals[3] - levelReader.range[2])
            };
        }

        // Read the initial state
        var state = new LevelState{
            objects = new int2s[levelReader.birds.Count + levelReader.frames.Count],
            fruitFresh = (1U << level.fruitCount) - 1
        };
        for (int i = 0; i < level.birdCount; i++)
        {
            state.objects[i] = new int2s{ 
                (levelReader.birds[i][0] - levelReader.range[0], 
                levelReader.birds[i][1] - levelReader.range[2]) };
            for (int j = 2; j < levelReader.birds[i].Length; j++)
            {
                state.objects[i].Add((
                    state.objects[i][j-2].Item1 + (levelReader.birds[i][j] == 0 ? 1 : levelReader.birds[i][j] == 1 ? -1 : 0),
                    state.objects[i][j-2].Item2 + (levelReader.birds[i][j] == 2 ? 1 : levelReader.birds[i][j] == 3 ? -1 : 0)
                ));
            }
        }
        for (int i = 0; i < levelReader.frames.Count; i++)
        {
            state.objects[level.birdCount + i] = new int2s{ 
                (levelReader.frames[i][0] - levelReader.range[0], 
                levelReader.frames[i][1] - levelReader.range[2]) };
            for (int j = 2; j < levelReader.frames[i].Length; j += 2)
            {
                state.objects[level.birdCount + i].Add((
                    state.objects[level.birdCount + i][0].Item1 + levelReader.frames[i][j],
                    state.objects[level.birdCount + i][0].Item2 + levelReader.frames[i][j+1]
                ));
            }
        }

#if !Use128Bit
        // Overflow check
        int bitUsed = level.posBit * (level.birdCount + level.frameCount) + level.fruitCount;
        for (int i = 0; i < level.birdCount; i++) bitUsed += state.objects[i].Count * 2;
        if (bitUsed + 4 >= sizeof(number) * 8) throw new Exception($"Too many bits used: {bitUsed}");
#endif
    
        // Solve the level
        var solver = new Solver{
            level = level,
            frameHeuristics = null,
            fruitMultiplier = levelReader.fruitMultiplier,
            frameMultiplier = levelReader.frameMultiplier,
            stepMultiplier = levelReader.stepMultiplier,
            targetMultiplier = levelReader.targetMultiplier,
            birdFrameMultiplier = levelReader.birdFrameMultiplier,
            allFruitAndFrameBonus = levelReader.allFruitAndFrameBonus,
        };
        if(levelReader.frameHeuristics.Count == level.frameCount * 2){
            solver.frameHeuristics = new int2[level.frameCount];
            for(int i = 0; i < level.frameCount; i++){
                solver.frameHeuristics[i] = (levelReader.frameHeuristics[i*2] - levelReader.range[0], levelReader.frameHeuristics[i*2+1] - levelReader.range[2]);
            }
        }
        solver.Solve(state);
    }
}

public class Solver{
    public LevelStatic level;
    public int2[] frameHeuristics;
    public int fruitMultiplier, frameMultiplier, stepMultiplier,
        allFruitAndFrameBonus, targetMultiplier, birdFrameMultiplier;
    private number initStateCache;
    private Dictionary<number, number> route; // Key: State, Value: Parent State << 4 | BirdIdx << 2 | Direction
    private PriorityQueue<LevelState, int> queue;
    private const string DIRECTIONS = "RLUD";
    private static int Distance(int2 a, int2 b){
        return Math.Abs(a.Item1 - b.Item1) + Math.Abs(a.Item2 - b.Item2);
    }
    private static int NonSymmetricDistance(int2 a, int2 b){
        return Math.Abs(a.Item1 - b.Item1) + 2 * Math.Max(b.Item2 - a.Item2, 0);
    }
    private int Hueristic(LevelState state){
        int h = state.step * stepMultiplier;

        int fruitCost = 0;
        if(fruitMultiplier != 0){
            for (int fruitIdx = 0; fruitIdx < level.fruitCount; fruitIdx++)
            {
                if (((state.fruitFresh >> fruitIdx) & 1) == 1)
                {
                    int minDist = int.MaxValue;
                    for (int birdIdx = 0; birdIdx < level.birdCount; birdIdx++)
                    {
                        if (state.objects[birdIdx] == null) continue;
                        int dist = Distance(state.objects[birdIdx][0], level.fruitPos[fruitIdx]);
                        if (minDist > dist) minDist = dist;
                    }
                    fruitCost += minDist;
                }
            }
            h += fruitCost * fruitMultiplier;
        }
        
        int frameTargetCost = 0;
        if(frameHeuristics != null && frameMultiplier != 0){
            for(int frameIdx = 0; frameIdx < level.frameCount; frameIdx++){
                frameTargetCost += Distance(state.objects[level.birdCount + frameIdx][0], frameHeuristics[frameIdx]);
            }
            h += frameTargetCost * frameMultiplier;
        }

        int birdTargetCost = 0;
        if (fruitCost == 0 && frameTargetCost == 0 || allFruitAndFrameBonus == 0)
        {
            for(int birdIdx = 0; birdIdx < level.birdCount; birdIdx++){
                if (state.objects[birdIdx] == null) continue;
                birdTargetCost += Distance(state.objects[birdIdx][0], level.target) * targetMultiplier;
            }
            h += birdTargetCost * targetMultiplier;
        }
        else h += allFruitAndFrameBonus;

        int birdFrameCost = 0;
        if (birdFrameMultiplier != 0 && frameTargetCost != 0){
            for(int birdIdx = 0; birdIdx < level.birdCount; birdIdx++){
                if (state.objects[birdIdx] == null) continue;
                birdFrameCost += Enumerable.Range(0, level.frameCount).Select(frameIdx =>
                    state.objects[level.birdCount + frameIdx]).Where(ls => ls != null).Min(ls =>
                        Distance(state.objects[birdIdx][0], ls[0]));
            }
            h += birdFrameCost * birdFrameMultiplier;
        }

        return h;
    }
    private static int2 Move(int2 pos, int dir){
        return (pos.Item1 + (dir == 0 ? 1 : dir == 1 ? -1 : 0), pos.Item2 + (dir == 2 ? 1 : dir == 3 ? -1 : 0));
    }
    private bool RecursiveTest(int[,] map, LevelState state, int idx, int dir, bool[] moveList){
        if (moveList[idx]) return true;
        moveList[idx] = true;
        foreach(int2 pos in state.objects[idx]){
            int2 nextPos = Move(pos, dir);
            if (nextPos.Item1 < 0 || nextPos.Item1 >= level.width ||
                nextPos.Item2 < 0 || nextPos.Item2 >= level.height) return false;
            int obj = map[nextPos.Item1, nextPos.Item2];
            if(obj >= LevelStatic.OBJECT_BASE){
                if(!moveList[obj - LevelStatic.OBJECT_BASE] &&
                    state.objects[obj - LevelStatic.OBJECT_BASE] != null && 
                    !RecursiveTest(map, state, obj - LevelStatic.OBJECT_BASE, dir, moveList)){
                    return false;
                }
            }
            else if(obj != LevelStatic.EMPTY){
                return false;
            }
        }
        return true;
    }

    /// <summary> `moveList`: no need to initialize, will be filled with `false` </summary>
    private bool TestSnakeMove(int[,] map, LevelState state, int idx, int dir, bool[] moveList){
        var nextPos = Move(state.objects[idx][0], dir);
        Array.Fill(moveList, false);
        if (map[nextPos.Item1, nextPos.Item2] >= LevelStatic.OBJECT_BASE){
            return RecursiveTest(map, state, map[nextPos.Item1, nextPos.Item2] - LevelStatic.OBJECT_BASE, dir, moveList) && !moveList[idx];
        }
        else
        { 
            return map[nextPos.Item1, nextPos.Item2] is >= LevelStatic.FRUIT_BASE or LevelStatic.EMPTY;
        }
    }
    /// <summary> `moveList`: no need to initialize, will be filled with `false` </summary>
    private bool TestEntityMove(int[,] map, LevelState state, int idx, int dir, bool[] moveList)
    {
        Array.Fill(moveList, false);
        moveList[idx] = true;
        foreach(var pos in state.objects[idx])
        {
            var nextPos = Move(pos, dir);
            if (map[nextPos.Item1, nextPos.Item2] >= LevelStatic.OBJECT_BASE ?
                !RecursiveTest(map, state, map[nextPos.Item1, nextPos.Item2] - LevelStatic.OBJECT_BASE, dir, moveList) :
                map[nextPos.Item1, nextPos.Item2] is >= LevelStatic.FRUIT_BASE or LevelStatic.WALL ||
                map[nextPos.Item1, nextPos.Item2] == LevelStatic.SPIKE && idx >= level.birdCount)
            {
                return false;
            }
        }
        return true;
    }

    private bool ApplyMove(int[,] map, LevelState state, int idx, int dir, bool snakeMove, bool[] moveList){
        if(snakeMove){
            // Strech head
            state.objects[idx].Insert(0, Move(state.objects[idx][0], dir));
            // If the bird falls out, report illegal
            if (state.objects[idx][0].Item2 == 0 ||
                state.objects[idx][0].Item2 == level.height - 1 ||
                state.objects[idx][0].Item1 == 0 ||
                state.objects[idx][0].Item1 == level.width - 1)
                return false;
            // Test Fruit
            int occupiedObj = map[state.objects[idx][0].Item1, state.objects[idx][0].Item2];
            if(occupiedObj is >= LevelStatic.FRUIT_BASE and < LevelStatic.OBJECT_BASE){ // Eat fruit, do not shrink tail
                state.fruitFresh &= ~(1U << (occupiedObj - LevelStatic.FRUIT_BASE));
            }
            else{ // Shrink tail
                map[state.objects[idx][^1].Item1, state.objects[idx][^1].Item2] = LevelStatic.EMPTY;
                state.objects[idx].RemoveAt(state.objects[idx].Count - 1);
            }
            // Test Portal
            if (level.portalPos != null && level.portalPos.Contains(state.objects[idx][0])){
                ApplyPortal(map, state, idx, state.objects[idx][0]);
            }
            // If the bird reaches target, remove it
            if (state.fruitFresh == 0 && state.objects[idx][0] == level.target)
            {
                state.objects[idx].ForEach(pos => map[pos.Item1, pos.Item2] = LevelStatic.EMPTY);
                state.objects[idx] = null;
            }
        }
        for (int objIdx = 0; objIdx < state.objects.Length; objIdx++){
            if(moveList[objIdx]){
                for(int objPartIdx = 0; objPartIdx < state.objects[objIdx].Count; objPartIdx++){
                    // Remove old position in map
                    map[state.objects[objIdx][objPartIdx].Item1, state.objects[objIdx][objPartIdx].Item2] = LevelStatic.EMPTY;
                    // Set new position
                    state.objects[objIdx][objPartIdx] = Move(state.objects[objIdx][objPartIdx], dir);
                    // If the bird reaches the target or the object falls out, remove it
                    if(objIdx < level.birdCount && 
                        objPartIdx == 0 && state.objects[objIdx][0] == level.target && state.fruitFresh == 0 ||
                        state.objects[objIdx][objPartIdx].Item2 == 0){
                        // If the bird or the needed frame falls out, report illegal
                        if (state.objects[objIdx][objPartIdx].Item2 == 0 && !(objIdx >= level.birdCount && frameHeuristics == null))
                            return false;
                        // Remove the rest old positions at once
                        for(int m = objPartIdx + 1; m < state.objects[objIdx].Count; m++){
                            map[state.objects[objIdx][m].Item1, state.objects[objIdx][m].Item2] = LevelStatic.EMPTY;
                        }
                        state.objects[objIdx] = null;
                        break;
                    }
                    // If the bird touches spike, report illegal
                    if(objIdx < level.birdCount &&
                        map[state.objects[objIdx][objPartIdx].Item1, state.objects[objIdx][objPartIdx].Item2] == LevelStatic.SPIKE)
                        return false;
                }
                // Test Portal
                if(level.portalPos != null && state.objects[objIdx] != null){
                    var portal = state.objects[objIdx].FirstOrDefault(pos => pos == level.portalPos[0] || pos == level.portalPos[1]);
                    if(portal != default && !state.objects[objIdx].Contains(Move(portal, dir))){
                        ApplyPortal(map, state, objIdx, portal);
                    }
                }
            }
        }
        // Update map
        for (int objIdx = 0; objIdx < state.objects.Length; objIdx++){
            if((moveList[objIdx] || snakeMove && objIdx == idx) && state.objects[objIdx] != null){
                for(int objPartIdx = 0; objPartIdx < state.objects[objIdx].Count; objPartIdx++){
                    map[state.objects[objIdx][objPartIdx].Item1, state.objects[objIdx][objPartIdx].Item2] = objIdx + LevelStatic.OBJECT_BASE;
                }
            }
        }
        return true;
    }
    private void ApplyPortal(int[,] map, LevelState state, int idx, int2 portal){
        // Test if the other portal is blocked
        int portalFlag = portal == level.portalPos[0] ? 0 : 1;
        int deltaX = level.portalPos[portalFlag ^ 1].Item1 - level.portalPos[portalFlag].Item1,
            deltaY = level.portalPos[portalFlag ^ 1].Item2 - level.portalPos[portalFlag].Item2;
        if (state.objects[idx].All(pos => map[pos.Item1 + deltaX, pos.Item2 + deltaY] == LevelStatic.EMPTY)){
            for (int partIdx = 0; partIdx < state.objects[idx].Count; partIdx++)
            {
                // Remove old object
                map[state.objects[idx][partIdx].Item1, state.objects[idx][partIdx].Item2] = LevelStatic.EMPTY;
                // Move to new place
                state.objects[idx][partIdx] = (
                    state.objects[idx][partIdx].Item1 + deltaX, 
                    state.objects[idx][partIdx].Item2 + deltaY);
            }
        }
    }

    private void CopyStateToMap(int[,] map, LevelState state){
        Array.Copy(level.initMap, map, level.initMap.Length);
        for(int objIdx = 0; objIdx < state.objects.Length; objIdx++){
            if (state.objects[objIdx] == null) continue;
            foreach (var pos in state.objects[objIdx]){
                map[pos.Item1, pos.Item2] = LevelStatic.OBJECT_BASE + objIdx;
            }
        }
        for(int fruitIdx = 0; fruitIdx < level.fruitCount; fruitIdx++){
            if(((state.fruitFresh >> fruitIdx) & 1) == 1){
                map[level.fruitPos[fruitIdx].Item1, level.fruitPos[fruitIdx].Item2] = LevelStatic.FRUIT_BASE + fruitIdx;
            }
        }
    }

    public void Solve(LevelState initState){
        initStateCache = initState.Serialize(level);
        route = new Dictionary<number, number>();
        queue = new PriorityQueue<LevelState, int>();
        queue.Enqueue(initState, Hueristic(initState));
        int[,] map = new int[level.width, level.height];
        bool[] moveList = new bool[initState.objects.Length], activeList = new bool[initState.objects.Length];
        bool mapPrinted = false;

        while(queue.Count > 0){
            var oldState = queue.Dequeue();
            for(int idx = 0; idx < level.birdCount; idx++){
                if (oldState.objects[idx] == null) continue;
                for (int dir = 0; dir < 4; dir++){
                    int2 nextPos = Move(oldState.objects[idx][0], dir);
                    // Pre-test illegal move
                    if(nextPos.Item1 < 0 || nextPos.Item1 >= level.width ||
                        nextPos.Item2 < 0 || nextPos.Item2 >= level.height ||
                        level.initMap[nextPos.Item1, nextPos.Item2] is LevelStatic.WALL or LevelStatic.SPIKE ||
                        nextPos == oldState.objects[idx][1]){
                        continue;
                    }

                    // Copy map and fill in object positions
                    CopyStateToMap(map, oldState);
                    if (!mapPrinted)
                    {
                        mapPrinted = true;
                        Console.WriteLine(PrintMap(map));
                    }

                    // Push
                    if (!TestSnakeMove(map, oldState, idx, dir, moveList)){
                        continue;
                    }
                    var newState = new LevelState{
                        objects = oldState.objects.Select(x => x == null ? null : x.ToList()).ToArray(),
                        fruitFresh = oldState.fruitFresh,
                        step = oldState.step + 1,
                    };
                    if(!ApplyMove(map, newState, idx, dir, true, moveList))
                        goto ILLEGAL_MOVE;

                    // Fall
                    Array.Fill(activeList, true);
                    while(true)
                    {
                        int fallIdx = Array.FindIndex(activeList, x => x);
                        if (fallIdx < 0)
                            break;
                        if (newState.objects[fallIdx] == null || !TestEntityMove(map, newState, fallIdx, 3, moveList))
                        {
                            activeList[fallIdx] = false;
                            continue;
                        }
                        if(!ApplyMove(map, newState, idx, 3, false, moveList))
                            goto ILLEGAL_MOVE;
                        Array.Fill(activeList, true);
                    }

                    // Reduce memory cost
                    for(int objIdx = 0; objIdx < newState.objects.Length; objIdx++){
                        if (newState.objects[objIdx] != null && Enumerable.SequenceEqual(newState.objects[objIdx], oldState.objects[objIdx])){
                            newState.objects[objIdx] = oldState.objects[objIdx];
                        }
                    }
                    
                    // Check if the level is solved
                    if (Enumerable.Range(0, level.birdCount).All(idx => newState.objects[idx] == null)){
                        // print solution
                        Console.WriteLine();
                        var ls = new Stack<int2>();
                        ls.Push((idx, dir));
#if PrintRoute
                            System.Console.WriteLine($"\nBefore [{idx+1}{DIRECTIONS[dir]}]:");
                            CopyStateToMap(map, oldState);
                            System.Console.WriteLine(PrintMap(map));
#endif
                        number key = oldState.Serialize(level);
                        while(initStateCache != key){
                            number value = route[key];
                            ls.Push(((int)((value >> 2) & 3), (int)(value & 3)));
                            key = value >> 4;
#if PrintRoute
                            System.Console.WriteLine($"\nBefore [{ls.Peek().Item1 + 1}{DIRECTIONS[ls.Peek().Item2]}]:");
                            newState.Deserialize(level, initState, key);
                            CopyStateToMap(map, newState);
                            System.Console.WriteLine(PrintMap(map));
#endif
                        }
                        Console.Write("Solution = ");
                        var sb = new StringBuilder();
                        int birdIdx = -1;
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
                        var serializedState = newState.Serialize(level);
                        if(!route.ContainsKey(serializedState)){
                            route.Add(serializedState, oldState.Serialize(level) << 4 | (number)(idx << 2 | dir));
                            queue.Enqueue(newState, Hueristic(newState));
                            
#if CheckIntermediate
                            if(route.Count % 100000 == 0)
                                Console.WriteLine(PrintMap(map)+$"Searched={route.Count}\nPending={queue.Count}\nHeuristic={Hueristic(newState)}");
#endif
                        }
                    }

                ILLEGAL_MOVE:;
                }
            }
        }
    }

#region DebugInfo
    public string PrintMap(int[,] map)
    {
        var sb = new StringBuilder();
        for (int i = level.height - 1; i >= 0; i--)
        {
            for (int j = 0; j < level.width; j++)
            {
                sb.Append(level.target.Item1 == j && level.target.Item2 == i ? 'O' :
                    level.portalPos != null && level.portalPos.Contains((j, i)) ? '@' :
                    (map[j, i] switch {
                    LevelStatic.WALL => '#',
                    LevelStatic.SPIKE => 'X',
                    >= LevelStatic.OBJECT_BASE => (char)('1' + map[j, i] - LevelStatic.OBJECT_BASE),
                    >= LevelStatic.FRUIT_BASE and < LevelStatic.OBJECT_BASE => (char)('a' + map[j, i] - LevelStatic.FRUIT_BASE),
                    _ => '_'
                }));
                sb.Append(' ');
            }
            sb.AppendLine();
        }
        return sb.ToString();
    }
#endregion
}