#include <iostream>
#include <benchmark/benchmark.h>

#include "source/util.h"
#include "source/AStar.hpp"
#include "source/AStar2.hpp"

const char filename[] = "/home/davide.faconti/Pictures/maze_1000.pgm";

static void BM_AStar_Otro_Tio(benchmark::State& state)
{
    AStar::Generator generator;
    Image image;
    ReadImageFromPGM(filename, &image);
    generator.setDiagonalMovement(true);
    generator.setWorldSize( {image.width, image.height } );

    for (int y=0; y < image.height; y++)
    {
        for (int x=0; x < image.width; x++)
        {
            if( image.at(x,y) < 100 )
            {
                generator.addCollision( {x,y} );
            }
        }
    }

    AStar::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath( { 1, 1 }, { image.width-3, image.height-3 }  );
    }

    for(auto& point: result)
    {
        image.at( point.x, point.y ) = 140;
    }

    WriteImageToPGM("/home/davide.faconti/Pictures/map_out.pgm", image);

}

static void BM_AStar_Davide(benchmark::State& state)
{
    AStar2::Generator generator;
    Image image;
    ReadImageFromPGM(filename, &image);

    generator.setWorldData( image.width, image.height, image.data.data() );

    AStar2::CoordinateList result;
    for (auto _ : state)
    {
        result = generator.findPath(
                    //{ 1, 1 }, { image.width-3, image.height-3 }
        { image.width/2, 0 }, { image.width/2, image.height/2 }
                    );
    }

    generator.exportPPM("/home/davide.faconti/Pictures/map_out_color.ppm", &result );

}

//BENCHMARK(BM_AStar_Otro_Tio);

BENCHMARK(BM_AStar_Davide);





BENCHMARK_MAIN();

