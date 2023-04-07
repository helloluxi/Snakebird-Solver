# Snakebird-Solver

A heuristic solver for the awesome puzzle game Snakebird.
All the star levels can be solved in seconds ~~if the heuristic parameters are properly adjusted~~.

## Requirements

+ .NET 7

## How to use

+ Fill in the level definitions and heuristic parameters in `level.json`. To describe a level, you should:
  + First, choose a square as the origin (0, 0);
  + The value of `range` is a 4-length int array that defines the coordinate range `x_min`, `x_max`, `y_min`, `y_max`;
  + The value of `birds` is an array of birds, each bird is again an int array that the first two elements are the coordinates of the bird head, and the rest elements define the direction of its body (0: right, 1: left, 2: up, 3: down) from head to tail;
  + The value of `target` is a 2-length int array that defines the coordinates of the target;
  + The value of `frames` is an array of iron frames, each frame is again an int array that the first two elements are the coordinates of the frame center, then every 2 elements define the relative coordinate of one of its body to the center;
  + The value of `fruits` is an int array, every 2 elements define a fruit's coordinate;
  + The value of `walls` and `spikes` are similar to `fruits`, but if a magic number 9999 occurs, then the following 4 elements define a rectangle of walls or spikes by `x_min`, `x_max`, `y_min`, `y_max`;
  + The value of `portals` is either an empty array or an array of 4-length int arrays, every 2 elements define a portal's coordinate;
  + The value of `heuristic` is a structure that defines the heuristic parameters, which is sensitive and may require efforts to adjust;
+ Run `dotnet run` to solve the level defined in `levels.json`, or run `dotnet run <level_file>` to solve the level defined in `<level_file>`;
+ The output looks like `Solution = [1]UR [2]LD ...`, which means the first bird should go up and then right, then the second bird should go left and then down, and so on.

## About the heuristic parameters

This part is quite important and tricky, here I just give some tips:

+ `useNonSymDistance`: likely to work better on levels with a high target;
+ `frameTargets`: an int array that every 2 elements define the target position of a frame where you think it should be when winning;
+ `extraCost`: usually a big number, for quickly finding the solution when all fruits are eaten and all frames are in position;
+ `birdFollowingFrameWeight`: which works only when the frames are not in position, to encourage the birds to push the frames;

## Options

In `SnakebirdSolver.csproj`, you can add the following options into `DefineConstants`:

+ `UseBigInt`: Use 128-bit integer instead of 64-bit integer to store a state, which is ~~usually~~ unnecessary;
+ `CheckIntermediate`: Print the intermediate states every 100000 searches, which is useful for adjusting the heuristic parameters by observing the search process;

## Notes

+ The level star-4 can be solved much faster if custom pruning is added (i.e. disallowing iron frames to touch the ground), but it's hard to write the pruning rules in json file;
+ Possible future work: store positions in cache just like shape;
+ You may need to manually select a larger value for `shapeKeyLength` is the error `Shape Key Overflow` or `Too many bits used` occurs;
