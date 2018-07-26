#include <iostream>
#include <benchmark/benchmark.h>

#include "util.h"
#include "source/AStar.hpp"
#include "source/AStar2.hpp"

const char filename[] = "/home/dfaconti/Pictures/map_1000.pgm";
const bool diagonal = false;

static void BM_AStar(benchmark::State& state)
{
  AStar::Generator generator;
  Image image;
  ReadImageFromPGM(filename, &image);

  generator.setDiagonalMovement(diagonal);
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
      result = generator.findPath( {0,0}, { image.width-1, image.height-1 } );
  }

  for(auto& point: result)
  {
        image.at( point.x, point.y ) = 140;
  }

  WriteImageToPGM("/home/dfaconti/Pictures/map_out.pgm", image);

}

static void BM_AStar2(benchmark::State& state)
{
  AStar2::Generator generator;
  generator.setDiagonalMovement(diagonal);
  Image image;
  ReadImageFromPGM(filename, &image);

  generator.setWorldData( image.width, image.height, image.data.data() );

  AStar2::CoordinateList result;
  for (auto _ : state)
  {
      result = generator.findPath( {0,0}, { image.width-1, image.height-1 } );
  }

  for(auto& point: result)
  {
        image.at( point.x, point.y ) = 140;
  }

  WriteImageToPGM("/home/dfaconti/Pictures/map_out2.pgm", image);

}

BENCHMARK(BM_AStar2);

//BENCHMARK(BM_AStar);




BENCHMARK_MAIN();

