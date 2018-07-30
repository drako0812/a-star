#include <iostream>
#include <benchmark/benchmark.h>

#include "source/util.h"
#include "source/AStar.hpp"
#include "source/AStar2.hpp"

static void BM_AStar_Other(benchmark::State& state)
{
    AStar::Generator generator;
    Image image;
    ReadImageFromPGM("../data/maze_250.pgm", &image);

    generator.setWorldSize( {image.width, image.height });

    for ( int y=0; y<image.height; y++ )
    for ( int x=0; x<image.width; x++ )
    {
        if(image.data[ x + y*image.width ] < 30)
        {
            generator.addCollision( { x,y } );
        }
    }

    for (auto _ : state)
    {
        generator.findPath(
        { image.width/2, 0 },
        { image.width/2, image.height/2 } );
    }
}


static void BM_AStar_A(benchmark::State& state)
{
    AStar2::Generator generator;
    Image image;
    ReadImageFromPGM("../data/maze_250.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );

    AStar2::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
        { image.width/2, 0 },
        { image.width/2, image.height/2 } );
    }
    generator.exportPPM("map_out_A.ppm", &result );
}


static void BM_AStar_B(benchmark::State& state)
{
    AStar2::Generator generator;
    Image image;
    ReadImageFromPGM("../data/maze_big.pgm", &image);
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

//BENCHMARK(BM_AStar_Other);
BENCHMARK(BM_AStar_A);
//BENCHMARK(BM_AStar_B);


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

