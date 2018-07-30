#include <iostream>
#include <benchmark/benchmark.h>

#include "source/util.h"
#include "source/AStar.hpp"
#include "source/AStar2.hpp"
#include "astar3.hpp"

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


static void BM_AStar3(benchmark::State& state)
{

    Image image;
    ReadImageFromPGM("../data/maze_big.pgm", &image);
    int W = image.width;
    int H = image.height;
    std::vector<float> weight( W*H, 0 );

    for ( int y=0; y < H; y++ )
    {
        for ( int x=0; x < W; x++ )
        {
            int index = x + y*W;
            if(image.data[ index ] < 30)
            {
                weight[ index ] = std::numeric_limits<float>::max();
            }
        }
    }

    int start_index = 1 + W ;
    int last_index  = W-3 + W*(H-3) ;

    std::vector<int> path( W*H, -1 );

    for (auto _ : state)
    {
        astar3( weight.data(), image.height, image.width,
                start_index,
                last_index,
                path.data() );
    }

    int index = last_index;
    while( index != start_index)
    {
        image.data[index] = 100;
        index = path[index];
    }

    WriteImageToPGM("map_out_big_other.ppm", image);
}



static void BM_AStar_Smooth_1000(benchmark::State& state)
{
    AStar2::Generator generator;
    Image image;
    ReadImageFromPGM("../data/maze_1000_smooth.pgm", &image);
    generator.setWorldData( image.width, image.height, image.data.data() );

    AStar2::CoordinateList result;
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
    generator.exportPPM("map_out_Big.ppm", &result );
}

BENCHMARK(BM_AStar3);
BENCHMARK(BM_AStar_Big);
//BENCHMARK(BM_AStar_A);


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

