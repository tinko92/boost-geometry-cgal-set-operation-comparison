This repository contains code for a program that compares the output of the set operations difference, symmetric difference, union and intersection between Boost.Geometry and CGAL for various test cases from Boost.Geometry. Specifically, this compares
- the CGAL Simple Cartesian Kernel with the CORE::Expr calculation type, which should be exact for all basic operations involving decimal numbers,
- the CGAL Epeck kernel, which should guarantee exact predicates and constructions for inputs with double coordinates,
- Boost Geometry point\_xy with double coordinates,
- Boost Geometry point\_xy with CORE::Expr coordinates.

System requirements are
- a GCC version that supports C++20
- CGAL, GMP and MPFR
- a checkout of Boost.Geometry in the directory above.

Tests can be run via
./make.sh
./run\_all.sh

Output can be browsed by running an HTTP server in out/, e.g.
cd out
python -m http.server 8000

Then visit 127.0.0.1:8000 with a web browser.

Some test cases fail because they are either invalid under the preconditions of CGAL or because they are not POLYGON/MULTIPOLYGON geometries.
