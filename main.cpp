#include <iostream>
#include <benchmark/benchmark.h>

#include "source/util.h"
#include "source/AStar2.hpp"

static void BM_AStar_A(benchmark::State& state)
{
    AStar2::Generator generator;
    Image image;
    ReadImageFromPGM("./data/maze_1000_smooth.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );

    AStar2::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { image.width/2, 0 },
        { image.width/2, image.height -1 } );
    }
    generator.exportPPM("map_out_A.ppm", &result );
}


static void BM_AStar_B(benchmark::State& state)
{
    AStar2::Generator generator;
    Image image;
    ReadImageFromPGM("./data/maze_big.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );

    AStar2::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { 1, 1 },
        { image.width-3, image.height-3 } );
    }
    generator.exportPPM("map_out_B.ppm", &result );
}

//BENCHMARK(BM_AStar_A);
BENCHMARK(BM_AStar_B);


BENCHMARK_MAIN();

