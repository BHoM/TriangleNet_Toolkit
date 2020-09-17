/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2020, the respective contributors. All rights reserved.
 *
 * Each contributor holds copyright over their respective contributions.
 * The project versioning (Git) records all such contribution source information.
 *                                           
 *                                                                              
 * The BHoM is free software: you can redistribute it and/or modify         
 * it under the terms of the GNU Lesser General Public License as published by  
 * the Free Software Foundation, either version 3.0 of the License, or          
 * (at your option) any later version.                                          
 *                                                                              
 * The BHoM is distributed in the hope that it will be useful,              
 * but WITHOUT ANY WARRANTY; without even the implied warranty of               
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                 
 * GNU Lesser General Public License for more details.                          
 *                                                                            
 * You should have received a copy of the GNU Lesser General Public License     
 * along with this code. If not, see <https://www.gnu.org/licenses/lgpl-3.0.html>.      
 */

using BH.oM.Geometry;
using BH.oM.Geometry.CoordinateSystem;
using BH.oM.Reflection.Attributes;
using System;
using System.Collections.Generic;
using System.Linq;
using BH.Engine.Geometry;
using BH.oM.Quantities.Attributes;

using System.ComponentModel;

namespace BH.Engine.Geometry.Triangulation
{
    public static partial class Compute
    {
        /***************************************************/
        /****      public Methods                       ****/
        /***************************************************/

        [Description("Creates a voronoi diagram from a list of coplanar, non-duplicate points. The returned polylines cells will correspond to the input points by index.")]
        [Input("points", "The coplanar points to use to generate the voronoi diagram. The algorithm can currently not handle colinear points.")]
        [Input("plane", "Optional plane for the voronoi. If provided, all points must be complanar with the plane. If nothing provided, a best fit plane will be calculated. For colinear points, if nothing no plane provided, a plane aligned with the global Z-axis will be created.")]
        [Input("tolerance", "Tolerance to be used in the method.", typeof(Length))]
        [Input("boundarySize", "To handle problems at boundaries, extra points are added outside the bounds of the provided points for the generation of the voronoi and then culled away. This value controls how far off these points should be created. If a zero or negative value is provided, this value will be calculated automatically, based on the size of the boundingbox of the provided points.", typeof(Length))]
        [Output("regions", "Voronoi regions calculated by the method. The position in the list will correspond to the position in the list of the provided points.")]
        public static List<Polyline> VoronoiRegions(List<Point> points, Plane plane = null, double boundarySize = -1, double tolerance = Tolerance.Distance)
        {
            List<Point> uniquePoints = points.CullDuplicates(tolerance);

            if (points.Count != uniquePoints.Count)
            {
                Reflection.Compute.RecordError("Some points are overlapping with others. Duplicates need to be culled out to create voronoi regions.");
                return new List<Polyline>();
            }

            //Preform check all inputs that triangulation can be done
            if (points == null || points.Count < 2)
            {
                Reflection.Compute.RecordError("Insuffient points for generating the diagram. Please provide at least 2 Points.");
                return new List<Polyline>();
            }

            //Special case for colinear points. No need to triangulate
            if (points.IsCollinear(tolerance))
            {
                return ColinearVoronoiRegions(points, plane, boundarySize, tolerance);
            }

            //Try fit plane if no is provided
            if (plane == null)
            {
                plane = points.FitPlane(tolerance);
            }

            if (plane == null)
            {
                Engine.Reflection.Compute.RecordError("Could not fit a plane through the Points and no plane was provided.");
                return new List<Polyline>();
            }

            //Check all points within distance of the plane
            if (points.Any(x => x.Distance(plane) > tolerance))
            {
                BH.Engine.Reflection.Compute.RecordError("Can only handle coplanar points. Please make sure all your points lie in the same plane.");
                return new List<Polyline>();
            }

            //Calculate the local coordinates
            Vector localX = points[1] - points[0];
            Vector localY = plane.Normal.CrossProduct(localX);
            Point min = points.Min();
            Point refPt = new Point { X = min.X, Y = min.Y };
            Cartesian localSystem = Create.CartesianCoordinateSystem(min, localX, localY);
            Cartesian globalSystem = Create.CartesianCoordinateSystem(refPt, Vector.XAxis, Vector.YAxis);

            //Transform to xy-plane
            List<Point> xyPoints = points.Select(x => x.Orient(localSystem, globalSystem)).ToList();


            //Add point at all corners. This is to try to handle weirdness at the boundaries
            BoundingBox bounds = xyPoints.Bounds();

            double lengthX;
            double lengthY;

            //If boundary size is or equal to 0, calculate based on the bounds
            if (boundarySize <= 0)
            {
                lengthX = (bounds.Max.X - bounds.Min.X);
                lengthY = (bounds.Max.X - bounds.Min.X);
            }
            else
            {
                //if positive, use value provided
                lengthX = boundarySize;
                lengthY = boundarySize;
            }

            double minX = bounds.Min.X - lengthX;
            double minY = bounds.Min.Y - lengthY;
            double maxX = bounds.Max.X + lengthX;
            double maxY = bounds.Max.Y + lengthY;

            xyPoints.Add(new Point { X = minX, Y = minY });
            xyPoints.Add(new Point { X = minX, Y = maxY });
            xyPoints.Add(new Point { X = maxX, Y = maxY });
            xyPoints.Add(new Point { X = maxX, Y = minY });

            //Convert to TringleNet vertecies
            List<TriangleNet.Geometry.Vertex> vertecies = xyPoints.Select(p => new TriangleNet.Geometry.Vertex(p.X, p.Y)).ToList();

            // Triangulate
            TriangleNet.Meshing.ITriangulator triangulator = new TriangleNet.Meshing.Algorithm.Dwyer();
            TriangleNet.Configuration config = new TriangleNet.Configuration();
            TriangleNet.Mesh mesh = (TriangleNet.Mesh)triangulator.Triangulate(vertecies, config);

            TriangleNet.Voronoi.StandardVoronoi voronoi = new TriangleNet.Voronoi.StandardVoronoi(mesh);

            List<Polyline> translatedPolylines = new List<Polyline>();
            double sqTol = tolerance * tolerance;

            // Convert regions to BHoMGeometry. Skip the last 4 faces as they correspond to the added boundary points.
            for (int i = 0; i < voronoi.Faces.Count - 4; i++)
            {
                var face = voronoi.Faces[i];
                try
                {
                    // List points defining the region
                    List<Point> pts = new List<Point>();
                    HashSet<int> visitedIds = new HashSet<int>();

                    int bailOut = 100000;
                    int counter = 0;

                    TriangleNet.Topology.DCEL.HalfEdge halfEdge = face.Edge;
                    int nextId;
                    do
                    {
                        visitedIds.Add(halfEdge.ID);
                        Point pt = new Point { X = halfEdge.Origin.X, Y = halfEdge.Origin.Y };
                        //Make sure two of the same points are not added. This could maybe be done in a more cleaver way with the half edge
                        if (pts.Count == 0 || pt.SquareDistance(pts.Last()) > sqTol)
                            pts.Add(pt);

                        halfEdge = halfEdge.Next;

                        if (halfEdge == null)
                            break;

                        nextId = halfEdge.ID;
                        counter++;
                    } while (!visitedIds.Contains(nextId) && counter < bailOut);

                    if (pts.Count > 0 && pts.First().SquareDistance(pts.Last()) > sqTol)
                        pts.Add(pts.First());

                    translatedPolylines.Add(BH.Engine.Geometry.Create.Polyline(pts));
                }
                catch (Exception)
                {
                    Engine.Reflection.Compute.RecordWarning("Failed to generate the region for at least one cell. An empty polyline as been added in its place.");
                    translatedPolylines.Add(new Polyline());
                }

            }

            //Orient back the regions to the local plane
            return translatedPolylines.Select(x => x.Orient(globalSystem, localSystem)).ToList();

        }

        /***************************************************/

        [Description("Creates a voronoi diagram from a list of coplanar, non-duplicate points and cuts the cells with the provided boundary curves. The returned polylines cells will correspond to the input points by index.")]
        [Input("points", "The coplanar points to use to generate the voronoi diagram. The algorithm can currently not handle colinear points.")]
        [Input("boundaryCurve", "Outer boundary for any of the voronoi cells. Must be coplanar with the provided points.")]
        [Input("openingCurves", "Inner openings to be cut out from the voronoi cells. Must be coplanar with the provided points.")]
        [Input("plane", "Optional plane for the voronoi. If provided, all points must be complanar with the plane. If nothing provided, a best fit plane will be calculated. For colinear points, if nothing no plane provided, a plane aligned with the global Z-axis will be created.")]
        [Input("boundarySize", "To handle problems at boundaries, extra points are added outside the bounds of the provided points for the generation of the voronoi and then culled away. This value controls how far off these points should be created. If a negative value is provided, this value will be calculated automatically, based on the size of the boundingbox of the provided points.", typeof(Length))]
        [Input("tolerance", "Tolerance to be used in the method.", typeof(Length))]
        [Output("regions", "Voronoi regions calculated by the method. The position in the list will correspond to the position in the list of the provided points.")]
        public static List<List<PolyCurve>> VoronoiRegions(List<Point> points, ICurve boundaryCurve, List<ICurve> openingCurves = null, Plane plane = null, double boundarySize = -1, double tolerance = Tolerance.Distance)
        {
            List<Point> uniquePoints = points.CullDuplicates(tolerance);
            openingCurves = openingCurves ?? new List<ICurve>();

            if (points.Count != uniquePoints.Count)
            {
                Reflection.Compute.RecordError("Some points are overlapping with others. Duplicates need to be culled out to create voronoi regions.");
                return new List<List<PolyCurve>>();
            }
            List<Point> checkingPoints = new List<Point>(points);
            checkingPoints.AddRange(boundaryCurve.IControlPoints());
            checkingPoints.AddRange(openingCurves.SelectMany(x => x.IControlPoints()));

            if (!checkingPoints.IsCoplanar(tolerance))
            {
                Engine.Reflection.Compute.RecordError("The points, boundaryCurve and openingCurves all need to be coplanar.");
                return new List<List<PolyCurve>>();
            }

            //Fit plane through boundary curve. This is to better support a linear array of points
            if (plane == null)
            {
                plane = boundaryCurve.IFitPlane(tolerance);
            }

            List<Polyline> untrimmedRegions = VoronoiRegions(points, plane, boundarySize, tolerance);

            List<List<PolyCurve>> boundaryTrimmedCurves = untrimmedRegions.Select(x => x.BooleanIntersection(boundaryCurve, tolerance)).ToList();

            List<List<PolyCurve>> trimmedCurves = new List<List<PolyCurve>>();

            foreach (List<PolyCurve> pointCurves in boundaryTrimmedCurves)
            {
                List<PolyCurve> curves = new List<PolyCurve>();

                foreach (PolyCurve curve in pointCurves)
                {
                    curves.AddRange(curve.BooleanDifference(openingCurves, tolerance));
                }
                trimmedCurves.Add(curves);
            }

            return trimmedCurves;
        }

        /***************************************************/
        /****      Private Methods                       ****/
        /***************************************************/

        [Description("Creates a set of voronoi cells from a list of colinear points. Requires the incoming points to be colinear to work.")]
        private static List<Polyline> ColinearVoronoiRegions(List<Point> points, Plane plane, double boundarySize, double tolerance)
        {
            List<Point> sorted = points.SortCollinear();
            Vector tan = sorted.Last() - sorted.First();
            if (plane == null)
            {
                Vector y = Vector.ZAxis.CrossProduct(tan);
                Vector normal = tan.CrossProduct(y).Normalise();
                plane = new Plane() { Origin = sorted.First(), Normal = normal };
            }

            //Check all points within distance of the plane
            if (points.Any(x => x.Distance(plane) > tolerance))
            {
                BH.Engine.Reflection.Compute.RecordError("Can only handle coplanar points, in the plane.");
                return new List<Polyline>();
            }

            Vector perp = plane.Normal.CrossProduct(tan).Normalise();

            //If boundary size is or equal to 0, calculate based on the bounds
            if (boundarySize <= 0)
            {
                double length = tan.Length() / 2;
                perp *= length;
            }
            else
            {
                //if positive, use value provided
                tan = tan.Normalise() * boundarySize;
                perp *= boundarySize / 2;
            }
            List<Point> divPts = new List<Point>();

            //Find average points
            for (int i = 0; i < sorted.Count - 1; i++)
            {
                Point mid = (sorted[i] + sorted[i + 1]) / 2;
                divPts.Add(mid);
            }

            List<Polyline> regions = new List<Polyline>();

            //Create the first region
            Point p1 = divPts.First() - tan;
            Point p2 = divPts.First();
            regions.Add(CreateRegion(p1, p2, perp));

            //Create central regions
            for (int i = 0; i < divPts.Count - 1; i++)
            {
                regions.Add(CreateRegion(divPts[i], divPts[i + 1], perp));
            }

            //Create last region
            p1 = divPts.Last();
            p2 = divPts.Last() + tan;
            regions.Add(CreateRegion(p1, p2, perp));

            //Sort back to follow original list
            Polyline[] sortedRegions = new Polyline[regions.Count];

            for (int i = 0; i < regions.Count; i++)
            {
                int index = points.IndexOf(sorted[i]);
                sortedRegions[index] = regions[i];
            }

            return sortedRegions.ToList();
        }

        /***************************************************/

        [Description("Creates a closed polyline quad region with the width of 2 times perp length, along this vector, where the mid1 and mid2 points will be on the middle of the width edges.")]
        private static Polyline CreateRegion(Point mid1, Point mid2, Vector perp)
        {
            Point p1 = mid1 + perp;
            Point p2 = mid1 - perp;
            Point p3 = mid2 - perp;
            Point p4 = mid2 + perp;

            List<Point> pts = new List<Point> { p1, p2, p3, p4, p1 };
            return new Polyline { ControlPoints = pts };
        }

        /***************************************************/
    }
}
