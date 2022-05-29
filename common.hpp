#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <exception>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <CGAL/CORE_Expr.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Polygon_set_2.h>

#include <algorithms/overlay/multi_overlay_cases.hpp>

using DKernel = CGAL::Exact_predicates_exact_constructions_kernel;
using DPoint_2 = DKernel::Point_2;
using DPolygon_2 = CGAL::Polygon_2<DKernel>;
using DPolygon_with_holes_2 = CGAL::Polygon_with_holes_2<DKernel>;
using DPolygon_set_2 = CGAL::Polygon_set_2<DKernel>;

using Real = CORE::Expr;
using RKernel = CGAL::Simple_cartesian<Real>;
using RPoint_2 = RKernel::Point_2;
using RPolygon_2 = CGAL::Polygon_2<RKernel>;
using RPolygon_with_holes_2 = CGAL::Polygon_with_holes_2<RKernel>;
using RPolygon_set_2 = CGAL::Polygon_set_2<RKernel>;

namespace bg = boost::geometry;
using point_real = bg::model::d2::point_xy<Real>;
using box_real = bg::model::box<point_real>;
using point_d = bg::model::d2::point_xy<double>;
using box_d = bg::model::box<point_d>;
using polygon_real = bg::model::polygon<point_real>;
using polygon_d = bg::model::polygon<point_d>;
using multi_polygon_real = bg::model::multi_polygon<polygon_real>;
using multi_polygon_d = bg::model::multi_polygon<polygon_d>;

template <typename CT> using point = bg::model::d2::point_xy<CT>;
template <typename CT> using polygon = bg::model::polygon<point<CT>>;
template <typename CT> using multi_polygon =
    bg::model::multi_polygon<polygon<CT>>;

template<typename OutRing>
OutRing to_cgal_ring(auto in)
{
    using OutPoint = typename OutRing::Point_2;
    bg::correct(in);
    OutRing r;
    for(int i = in.size() - 1; i > 0; --i)
        if(in[i].x() != in[i - 1].x() || in[i].y() != in[i - 1].y() )
            r.push_back(OutPoint(in[i].x(), in[i].y()));
    return r;
}

template <typename PS2, typename Poly>
PS2 to_cgal(Poly poly) requires std::is_same_v<typename bg::tag<Poly>::type, bg::polygon_tag>
{
    using P2 = typename PS2::Polygon_2;
    PS2 out;
    out.join(to_cgal_ring<P2>(poly.outer()));
    for(const auto& hole : poly.inners())
    {
        out.difference(to_cgal_ring<P2>(hole));
    }
    return out;
}

template <typename PS2, typename MP>
PS2 to_cgal(MP multi_poly) requires std::is_same_v<typename bg::tag<MP>::type, bg::multi_polygon_tag>
{
    PS2 out;
    for(const auto& poly : multi_poly)
        out.join(to_cgal<PS2>(poly));
    return out;
}

auto pwh_area(auto pwh)
{
    auto outerP = pwh.outer_boundary();
    auto result = outerP.area();
    if (! pwh.has_holes())
        return result;
    auto hit = pwh.holes_begin();
    while (hit != pwh.holes_end()) {
        auto curHoleArea= (*hit).area();
        result = result + curHoleArea;
        ++hit;
    }
    return result;
}

template <typename PWH>
auto ps_area(auto ps)
{
    std::vector<PWH> pwhs;
    ps.polygons_with_holes(std::back_inserter(pwhs));
    decltype(pwh_area(PWH())) area(0);
    if(pwhs.size() == 0) return area;
    for(const auto& pwh : pwhs)
       area += pwh_area(pwh);
    return area;
}

template <typename CT>
void bg_to_svg(const multi_polygon<CT>& mp, auto& out, std::string title, std::string style, auto box)
{
    out << "<?xml version=\"1.0\" standalone=\"no\"?>\n"
        << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n"
        << "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
        << "<svg  version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"";
    if(mp.size() == 0)
    {
        out << "0 0 0 0\"></svg>";
        return;
    }
    out << 0 << " " << 0 << " " << (box.max_corner().x() - box.min_corner().x()) << " " << (box.max_corner().y() - box.min_corner().y()) << "\">\n";
    out << "<g title=\"" << title << "\" fill-rule=\"evenodd\">\n";
    for (const auto& poly: mp)
    {
        out << "<path d=\"";
        const auto& outer = poly.outer();
        out << "M ";
        for(int i = 0; i < outer.size() - 1; ++i)
        {
            const auto& p = outer[i];
            out << (p.x() - box.min_corner().x()) << "," << (p.y() - box.min_corner().y()) << " L ";
        }
        
        out << (outer.back().x() - box.min_corner().x()) << "," << (outer.back().y() - box.min_corner().y()) << " ";
        for(const auto& hole : poly.inners())
        {
            out << "M ";
            for(int i = 0; i < hole.size() - 1; ++i)
            {
                const auto& p = hole[i];
                out << (p.x() - box.min_corner().x()) << "," << (p.y() - box.min_corner().y()) << " L ";
            }
            out << (hole.back().x() - box.min_corner().x()) << "," << (hole.back().y() - box.min_corner().y()) << " ";
        }
        out << "z\" style=\"" << style << "\" />\n";
    }
    out << "</g></svg>";
}

template <typename PS2>
void ps_to_svg(const PS2& ps2, auto& out, std::string title, std::string style, auto box)
{
    out << "<?xml version=\"1.0\" standalone=\"no\"?>\n"
        << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n"
        << "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n"
        << "<svg  version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"";
    using PWH = typename PS2::Polygon_with_holes_2;
    std::vector<PWH> pwhs;
    ps2.polygons_with_holes(std::back_inserter(pwhs));
    if(pwhs.size() == 0)
    {
        out << "0 0 0 0\"></svg>";
        return;
    }
    auto bbox = pwhs.front().bbox();
    for(const auto& pwh : pwhs)
    {
        bbox += pwh.bbox();
    }
    //out << box.min_corner().x() << " " << box.min_corner().y() << " " << box.max_corner().x() << " " << box.max_corner().y() << "\">\n";
    out << 0 << " " << 0 << " " << (box.max_corner().x() - box.min_corner().x()) << " " << (box.max_corner().y() - box.min_corner().y()) << "\">\n";
    out << "<g title=\"" << title << "\" fill-rule=\"evenodd\">\n";
    for (const auto& pwh : pwhs)
    {
        out << "<path d=\"";
        const auto& outer = pwh.outer_boundary();
        out << "M ";
        for(const auto& p : outer)
            out << (p.x() - box.min_corner().x()) << "," << (p.y() - box.min_corner().y()) << " L ";
        out << (outer.begin()->x() - box.min_corner().x()) << "," << (outer.begin()->y() - box.min_corner().y()) << " ";
        for(const auto& hole : pwh.holes())
        {
            out << "M ";
            for(const auto& p : hole)
                out << (p.x() - box.min_corner().x()) << "," << (p.y() - box.min_corner().y()) << " L ";
            out << (hole.begin()->x() - box.min_corner().x()) << "," << (hole.begin()->y() - box.min_corner().y()) << " ";
        }
        out << "z\" style=\"" << style << "\" />\n";
    }
    out << "</g></svg>";
}

template <template <typename> class Geo1, template <typename> class Geo2>
void test_op(std::string id, const std::string input[2], std::string op)
{
    id += std::string("_")+op;
    std::cout << id << "; " << input[0] << "; " << input[1] << "; " << std::flush;
    using geo1_real = Geo1<Real>;
    using geo2_real = Geo2<Real>;
    using geo1_d = Geo1<double>;
    using geo2_d = Geo2<double>;
    auto a_real = bg::from_wkt<geo1_real>(input[0]);
    auto b_real = bg::from_wkt<geo2_real>(input[1]);
    bg::correct(a_real);
    bg::correct(b_real);

    auto a_d = bg::from_wkt<geo1_d>(input[0]);
    auto b_d = bg::from_wkt<geo2_d>(input[1]);
    bg::correct(a_d);
    bg::correct(b_d);
    auto ea = bg::return_envelope<box_d>(a_d);
    auto eb = bg::return_envelope<box_d>(b_d);
    auto env_d = box_d(point_d(
                    std::min(ea.min_corner().x(), eb.min_corner().x()),
                    std::min(ea.min_corner().y(), eb.min_corner().y())),
                point_d(
                    std::max(ea.max_corner().x(), eb.max_corner().x()),
                    std::max(ea.max_corner().y(), eb.max_corner().y())));
    auto max_dim = std::max(env_d.max_corner().x() - env_d.min_corner().x(),
                            env_d.max_corner().y() - env_d.min_corner().y());
    multi_polygon_d diff_d;
    multi_polygon_real diff_real;
    if (op == std::string("sym_difference"))
    {
        bg::sym_difference(a_d, b_d, diff_d);
        bg::sym_difference(a_real, b_real, diff_real);
    }
    else if(op == std::string("difference"))
    {
        bg::difference(a_d, b_d, diff_d);
        bg::difference(a_real, b_real, diff_real);
    }
    else if(op == std::string("union"))
    {
        bg::union_(a_d, b_d, diff_d);
        bg::union_(a_real, b_real, diff_real);
    }
    else if(op == std::string("intersection"))
    {
        bg::intersection(a_d, b_d, diff_d);
        bg::intersection(a_real, b_real, diff_real);
    }

    std::string viewbox("viewBox=\"0 0 300 300\"");

    std::ofstream svg_problem_d(std::string("out/")+id+std::string("_problem_double.svg"));
    boost::geometry::svg_mapper<point_d> mapper_problem_d(svg_problem_d, 300, 300, 1, viewbox);
    mapper_problem_d.add(a_d);
    mapper_problem_d.add(b_d);
    mapper_problem_d.map(a_d, "fill-opacity:0.3;fill:rgb(255,0,0);stroke:rgb(0,0,0);stroke-width:1");
    mapper_problem_d.map(b_d, "fill-opacity:0.3;fill:rgb(0,255,0);stroke:rgb(0,0,0);stroke-width:1");

    std::ofstream svg_problem(std::string("out/")+id+std::string("_problem_real.svg"));
    boost::geometry::svg_mapper<point_real, true, Real> mapper_problem(svg_problem, 300, 300, 1, viewbox);
    mapper_problem.add(a_real);
    mapper_problem.add(b_real);
    mapper_problem.map(a_real, "fill-opacity:0.3;fill:rgb(255,0,0);stroke:rgb(0,0,0);stroke-width:1");
    mapper_problem.map(b_real, "fill-opacity:0.3;fill:rgb(0,255,0);stroke:rgb(0,0,0);stroke-width:1");

    std::ofstream svg_bg_res(std::string("out/")+id+std::string("_bg_res.svg"));
    boost::geometry::svg_mapper<point_d> mapper_bg_res(svg_bg_res, 300, 300, 1, viewbox);
    mapper_bg_res.add(a_d);
    mapper_bg_res.add(b_d);
    mapper_bg_res.map(diff_d, "fill-opacity:0.3;fill:rgb(0,0,255);stroke:rgb(51,51,153);stroke-width:1");

    std::ofstream svg_bg_res_real(std::string("out/")+id+std::string("_bg_res_real.svg"));
    bg_to_svg(diff_real, svg_bg_res_real, std::string("A + B (CGAL EPECK)"), std::string("fill-opacity:0.3;fill:rgb(0,0,255);stroke:rgb(51,51,153);stroke-width:")+std::to_string(max_dim / 400.), env_d);
    try
    {
        RPolygon_set_2 true_res_r;
        true_res_r.join(to_cgal<RPolygon_set_2>(a_real));
        DPolygon_set_2 true_res_d;
        true_res_d.join(to_cgal<DPolygon_set_2>(a_d));
        if(op == std::string("sym_difference"))
        {
            true_res_r.symmetric_difference(to_cgal<RPolygon_set_2>(b_real));
            true_res_d.symmetric_difference(to_cgal<DPolygon_set_2>(b_d));
        }
        else if(op == std::string("difference"))
        {
            true_res_r.difference(to_cgal<RPolygon_set_2>(b_real));
            true_res_d.difference(to_cgal<DPolygon_set_2>(b_d));
        }
        else if(op == std::string("union"))
        {
            true_res_r.join(to_cgal<RPolygon_set_2>(b_real));
            true_res_d.join(to_cgal<DPolygon_set_2>(b_d));
        }
        else if(op == std::string("intersection"))
        {
            true_res_r.intersection(to_cgal<RPolygon_set_2>(b_real));
            true_res_d.intersection(to_cgal<DPolygon_set_2>(b_d));
        }
        auto true_res_r_area = ps_area<RPolygon_with_holes_2>(true_res_r);
        auto true_res_d_area = ps_area<DPolygon_with_holes_2>(true_res_d);

        RPolygon_set_2 err_r = true_res_r;
        DPolygon_set_2 err_d = true_res_d;

        err_r.symmetric_difference(to_cgal<RPolygon_set_2>(diff_real));
        err_d.symmetric_difference(to_cgal<DPolygon_set_2>(diff_d));

        auto err_r_area = ps_area<RPolygon_with_holes_2>(err_r);
        auto err_d_area = ps_area<DPolygon_with_holes_2>(err_d);
        std::cout << true_res_r.number_of_polygons_with_holes() << "; " 
                  << true_res_r_area << "; "
                  << true_res_d.number_of_polygons_with_holes() << "; "
                  << true_res_d_area    << "; "
                  << diff_d.size()      << "; "
                  << bg::area(diff_d)   << "; "
                  << err_r_area         << "; "
                  << err_d_area         << "; "
#ifdef CGAL_NO_PRECONDITIONS
                  << "CGAL NO PRECONDITION"
#else
                  << "OK"
#endif
                  << std::endl;

        std::ofstream svg_epeck_alt(std::string("out/")+id+std::string("_epeck_res_alt.svg"));
        ps_to_svg(true_res_d, svg_epeck_alt, std::string("A + B (CGAL EPECK)"), std::string("fill-opacity:0.3;fill:rgb(0,0,255);stroke:rgb(51,51,153);stroke-width:")+std::to_string(max_dim / 400.), env_d);

        std::ofstream svg_real_alt(std::string("out/")+id+std::string("_real_res_alt.svg"));
        ps_to_svg(true_res_r, svg_real_alt, std::string("A + B (CGAL Real)"), std::string("fill-opacity:0.3;fill:rgb(0,0,255);stroke:rgb(51,51,153);stroke-width:")+std::to_string(max_dim / 400.), env_d);
    }
    catch(const std::exception& e)
    {
        std::string error(e.what());
        error.erase(std::remove(error.begin(), error.end(), '\n'), error.end());
        std::cout << -1 << "; " 
                  << -1 << "; "
                  << -1 << "; "
                  << -1 << "; "
                  << -1 << "; "
                  << -1 << "; "
                  << -1 << "; "
                  << -1 << "; "
                  << error << std::endl;
        std::cerr << e.what() << "\n";
    }
}

#define TEST_OP_MULTI(caseid) \
    if(std::string(argv[1])==std::string(#caseid)) test_op<multi_polygon, multi_polygon>(#caseid, caseid,std::string(argv[2]))

#define TEST_OP(caseid) \
    if(std::string(argv[1])==std::string(#caseid)) test_op<polygon, polygon>(#caseid, caseid, std::string(argv[2]))
