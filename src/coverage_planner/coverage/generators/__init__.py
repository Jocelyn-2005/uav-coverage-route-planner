"""Interchangeable geometric coverage-structure generators."""

from coverage_planner.coverage.generators.base import CoverageStructureGenerator
from coverage_planner.coverage.generators.bcd import BCDGenerator, decompose_boustrophedon_cells
from coverage_planner.coverage.generators.scanline_clipped import ScanlineClippedGenerator

__all__ = [
    "BCDGenerator",
    "CoverageStructureGenerator",
    "ScanlineClippedGenerator",
    "decompose_boustrophedon_cells",
]
