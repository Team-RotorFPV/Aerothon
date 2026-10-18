// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright 2026 Team Rotor FPV
// Uses the dynamically linked Karto API supplied by slam_toolbox 2.8.5.
#include "slam_toolbox/slam_mapper.hpp"

namespace {
class InclusiveGrid : public karto::OccupancyGrid {
public:
  using karto::OccupancyGrid::OccupancyGrid;

  static karto::OccupancyGrid * build(const karto::LocalizedRangeScanVector & scans,
                                    double resolution, kt_int32u passes, double threshold)
  {
    if (scans.empty()) return nullptr;
    kt_int32s width, height;
    karto::Vector2<kt_double> offset;
    ComputeDimensions(scans, resolution, width, height, offset);
    // WorldToGrid rounds maximum endpoints to width/height. Those indices
    // require one additional cell, otherwise two outer walls are discarded.
    auto * grid = new InclusiveGrid(width + 1, height + 1, offset, resolution);
    grid->SetMinPassThrough(passes);
    grid->SetOccupancyThreshold(threshold);
    grid->CreateFromScans(scans);
    return grid;
  }
};
}

namespace mapper_utils {
karto::OccupancyGrid * SMapper::getOccupancyGrid(const double & resolution)
{
  auto * mapper = getMapper();
  return InclusiveGrid::build(mapper->GetAllProcessedScans(), resolution,
      static_cast<kt_int32u>(mapper->getParamMinPassThrough()),
      static_cast<kt_double>(mapper->getParamOccupancyThreshold()));
}
}
