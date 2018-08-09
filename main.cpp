#include <iostream>
#include <benchmark/benchmark.h>

#include "source/util.h"
#include "source/AStar2.hpp"


static void BM_AStar_Smooth_1000(benchmark::State& state)
{
    AStar::PathFinder generator;
    Image image;
    ReadImageFromPGM("../data/maze_1000_smooth.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );
    generator.allow5by5(true);

    AStar::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { image.width/2, 0 },
        { image.width/2, image.height -1 } );
    }
    generator.exportPPM("map_out_smooth.ppm", &result );
}


static void BM_AStar_Big(benchmark::State& state)
{
    AStar::PathFinder generator;
    Image image;
    ReadImageFromPGM("../data/maze_big.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );

    AStar::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { 1, 1 },
        { image.width-3, image.height-3 } );
    }
    generator.exportPPM("map_out_Big.ppm", &result );
}


BENCHMARK(BM_AStar_Big);
BENCHMARK(BM_AStar_Smooth_1000);


BENCHMARK_MAIN();

//int main()
//{
//    AStar2::Generator generator;
//    Image image;
//    ReadImageFromPGM("../data/maze_1000_smooth.pgm", &image);
//    generator.setWorldData( image.width, image.height, image.data.data() );

//    AStar2::CoordinateList         result = generator.findPath(
//    { image.width/2, 0 },
//    { image.width/2, image.height -1 } );

//    //generator.exportPPM("map_out_B.ppm", &result );
//    return 0;
//}

