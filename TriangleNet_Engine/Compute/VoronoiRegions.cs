/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2025, the respective contributors. All rights reserved.
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
using BH.oM.Base;
using BH.oM.Base.Attributes;
using System;
using System.Collections.Generic;
using System.Linq;
using BH.Engine.Geometry;
using BH.oM.Quantities.Attributes;
using BH.oM.Data.Collections;
using BH.Engine.Data;

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
        [Input("boundarySize", "To handle problems at boundaries, extra points are added outside the bounds of the provided points for the generation of the voronoi and then culled away. This value controls how far off these points should be created. If a zero or negative value is provided, this value will be calculated automatically, based on the size of the boundingbox of the provided points.", typeof(Length))]
        [Input("tolerance", "Tolerance to be used in the method.", typeof(Length))]
        [Output("regions", "Voronoi regions calculated by the method. The position in the list will correspond to the position in the list of the provided points.")]
        public static List<Polyline> VoronoiRegions(List<Point> points, Plane plane = null, double boundarySize = -1, double tolerance = Tolerance.Distance)
        {
            List<Point> uniquePoints = points.CullDuplicates(tolerance);

            if (points.Count != uniquePoints.Count)
            {
                Base.Compute.RecordError("Some points are overlapping with others. Duplicates need to be culled out to create voronoi regions.");
                return new List<Polyline>();
            }

            //Preform check all inputs that triangulation can be done
            if (points == null || points.Count < 2)
            {
                Base.Compute.RecordError("Insuffient points for generating the diagram. Please provide at least 2 Points.");
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
                Engine.Base.Compute.RecordError("Could not fit a plane through the Points and no plane was provided.");
                return new List<Polyline>();
            }

            //Check all points within distance of the plane
            if (points.Any(x => x.Distance(plane) > tolerance))
            {
                BH.Engine.Base.Compute.RecordError("Can only handle coplanar points. Please make sure all your points lie in the same plane.");
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
            TransformMatrix toGlobal = Create.OrientationMatrix(localSystem, globalSystem);
            List<Point> xyPoints = points.Select(x => x.Transform(toGlobal)).ToList();


            //Add point at all corners. This is to try to handle weirdness at the boundaries
            BoundingBox bounds = xyPoints.Bounds();

            double lengthX;
            double lengthY;

            //If boundary size is or equal to 0, calculate based on the bounds
            if (boundarySize <= 0)
            {
                lengthX = (bounds.Max.X - bounds.Min.X);
                lengthY = (bounds.Max.Y - bounds.Min.Y);
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
                    Engine.Base.Compute.RecordWarning("Failed to generate the region for at least one cell. An empty polyline as been added in its place.");
                    translatedPolylines.Add(new Polyline());
                }

            }

            //Orient back the regions to the local plane
            TransformMatrix toLocal = Geometry.Create.OrientationMatrix(globalSystem, localSystem);
            return translatedPolylines.Select(x => x.Transform(toLocal)).ToList();

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
                Base.Compute.RecordError("Some points are overlapping with others. Duplicates need to be culled out to create voronoi regions.");
                return new List<List<PolyCurve>>();
            }
            List<Point> checkingPoints = new List<Point>(points);
            checkingPoints.AddRange(boundaryCurve.IControlPoints());
            checkingPoints.AddRange(openingCurves.SelectMany(x => x.IControlPoints()));

            if (!checkingPoints.IsCoplanar(tolerance))
            {
                Engine.Base.Compute.RecordError("The points, boundaryCurve and openingCurves all need to be coplanar.");
                return new List<List<PolyCurve>>();
            }

            //Fit plane through boundary curve. This is to better support a linear array of points
            if (plane == null && boundaryCurve != null)
            {
                plane = boundaryCurve.IFitPlane(tolerance);
            }
            if (boundarySize < 0 && boundaryCurve != null)
            {
                BoundingBox boundaryBox = boundaryCurve.IBounds();
                boundarySize = Math.Max(boundaryBox.HorizontalHypotenuseLength(), boundaryBox.Height());
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

        [Description("Creates a voronoi diagram from a list of coplanar, non-duplicate points and curves. The returned polylines cells will correspond to the input points and curves by index.")]
        [Input("points", "The coplanar points to use to generate the voronoi diagram. The algorithm can currently not handle colinear points.")]
        [Input("curves", "The coplanar curves to use to generate the voronoi diagram. The algorithm can currently not handle colinear points.")]
        [Input("curveDiscretisation", "Parameter defining how many points the curves should be discretised to be used when creating the Voronoi diagram.")]
        [Input("plane", "Optional plane for the voronoi. If provided, all points must be complanar with the plane. If nothing provided, a best fit plane will be calculated. For colinear points, if nothing no plane provided, a plane aligned with the global Z-axis will be created.")]
        [Input("boundarySize", "To handle problems at boundaries, extra points are added outside the bounds of the provided points for the generation of the voronoi and then culled away. This value controls how far off these points should be created. If a negative value is provided, this value will be calculated automatically, based on the size of the boundingbox of the provided points and curves.", typeof(Length))]
        [Input("tolerance", "Tolerance to be used in the method.", typeof(Length))]
        [Input("simplifyDistTol", "Distance tolerance to be used for simplifying the regions, reducing the number of controlpoints.", typeof(Length))]
        [Input("simplifyAngTol", "Angle tolerance to be used for simplifying the regions, reducing the number of controlpoints.", typeof(Angle))]
        [MultiOutput(0, "pointRegions", "Voronoi regions calculated by the method. The position in the list will correspond to the position in the list of the provided points.")]
        [MultiOutput(1, "curveRegions", "Voronoi regions calculated by the method. The position in the list will correspond to the position in the list of the provided lines.")]
        public static Output<List<Polyline>, List<List<Polyline>>> VoronoiRegions(List<Point> points = null, List<ICurve> curves = null, int curveDiscretisation = 10, Plane plane = null, double boundarySize = -1, double tolerance = Tolerance.Distance, double simplifyDistTol = Tolerance.MacroDistance, double simplifyAngTol = Math.PI / 180 * 3)
        {
            points = points ?? new List<Point>();
            curves = curves ?? new List<ICurve>();

            if (points.Count == 0 && curves.Count == 0)
            {
                Engine.Base.Compute.RecordWarning("Require either points or lines to compute Voronoi regions.");
                return new Output<List<Polyline>, List<List<Polyline>>>() { Item1 = new List<Polyline>(), Item2 = new List<List<Polyline>>()};
            }

            List<Point> voronoiPts = new List<Point>(points);
            List<bool> startOffset, endOffset;
            List<List<Point>> mustIncludePts;
            if (AnyIntersectingNotEndPoints(curves, points, out startOffset, out endOffset, out mustIncludePts, tolerance))
            {
                Engine.Base.Compute.RecordError("Some of the curves are intersecting. Please split up any intersecting curves.");
                return new Output<List<Polyline>, List<List<Polyline>>>() { Item1 = new List<Polyline>(), Item2 = new List<List<Polyline>>() };
            }

            List<Tuple<ICurve, int, int>> linePtIndecies = new List<Tuple<ICurve, int, int>>();
            double sqTol = tolerance * tolerance;
            for (int i = 0; i < curves.Count; i++)
            {
                IEnumerable<ICurve> subCurves = curves[i].ISubParts();
                List<Point> curvePts = subCurves.SelectMany(x => x.SamplePoints(curveDiscretisation)).ToList();

                if (startOffset[i])
                    curvePts[0] = curvePts[0] + subCurves.First().IStartDir() * tolerance * 5;

                if (endOffset[i])
                    curvePts[curvePts.Count-1] = curvePts[curvePts.Count - 1] - subCurves.Last().IEndDir() * tolerance * 5;

                curvePts.AddRange(mustIncludePts[i]);

                curvePts = curvePts.CullDuplicates(tolerance);
                linePtIndecies.Add(new Tuple<ICurve, int, int>(curves[i], voronoiPts.Count, curvePts.Count));
                voronoiPts.AddRange(curvePts);
            }

            List<Point> uniquePoints = voronoiPts.CullDuplicates(tolerance);

            if (voronoiPts.Count != uniquePoints.Count)
            {
                Base.Compute.RecordError("Some points are overlapping with others. Duplicates need to be culled out to create voronoi regions.");
                return new Output<List<Polyline>, List<List<Polyline>>>() { Item1 = new List<Polyline>(), Item2 = new List<List<Polyline>>() };
            }

            if (!voronoiPts.IsCoplanar(tolerance))
            {
                Engine.Base.Compute.RecordError("The points, boundaryCurve and openingCurves all need to be coplanar.");
                return new Output<List<Polyline>, List<List<Polyline>>>() { Item1 = new List<Polyline>(), Item2 = new List<List<Polyline>>() };
            }

            List<Polyline> voronoiRegions = VoronoiRegions(voronoiPts, plane, boundarySize, tolerance);

            List<Polyline> pointRegions = voronoiRegions.GetRange(0, points.Count);
            List<List<Polyline>> curveRegions = new List<List<Polyline>>();

            foreach (var lineIndecies in linePtIndecies)
            {
                List<Polyline> lineItemRegions = voronoiRegions.GetRange(lineIndecies.Item2, lineIndecies.Item3);
                curveRegions.Add(SimpleBooleanUnion(lineItemRegions));
            }

            return new Output<List<Polyline>, List<List<Polyline>>>
            {
                Item1 = pointRegions.Select(x => x.Simplify(simplifyDistTol, simplifyAngTol)).ToList(),
                Item2 = curveRegions.Select(x => x.Select(y => y.Simplify(simplifyDistTol, simplifyAngTol)).ToList()).ToList()
            };
        }

        /***************************************************/

        [Description("Creates a voronoi diagram from a list of coplanar, non-duplicate points and curves and cuts the cells with the provided boundary curves. The returned polylines cells will correspond to the input points and curves by index.")]
        [Input("points", "The coplanar points to use to generate the voronoi diagram. The algorithm can currently not handle colinear points.")]
        [Input("curves", "The coplanar curves to use to generate the voronoi diagram. The algorithm can currently not handle colinear points.")]
        [Input("curveDiscretisation", "Parameter defining how many points the curves should be discretised to be used when creating the Voronoi diagram.")]
        [Input("boundaryCurve", "Outer boundary for any of the voronoi cells. Must be coplanar with the provided points and curves.")]
        [Input("openingCurves", "Inner openings to be cut out from the voronoi cells. Must be coplanar with the provided points and curves.")]
        [Input("plane", "Optional plane for the voronoi. If provided, all points must be complanar with the plane. If nothing provided, a best fit plane will be calculated. For colinear points, if nothing no plane provided, a plane aligned with the global Z-axis will be created.")]
        [Input("boundarySize", "To handle problems at boundaries, extra points are added outside the bounds of the provided points for the generation of the voronoi and then culled away. This value controls how far off these points should be created. If a negative value is provided, this value will be calculated automatically, based on the size of the boundingbox of the boundary curve.", typeof(Length))]
        [Input("tolerance", "Tolerance to be used in the method.", typeof(Length))]
        [Input("simplifyDistTol", "Distance tolerance to be used for simplifying the regions, reducing the number of controlpoints.", typeof(Length))]
        [Input("simplifyAngTol", "Angle tolerance to be used for simplifying the regions, reducing the number of controlpoints.", typeof(Angle))]
        [MultiOutput(0, "pointRegions", "Voronoi regions calculated by the method. The position in the list will correspond to the position in the list of the provided points.")]
        [MultiOutput(1, "curveRegions", "Voronoi regions calculated by the method. The position in the list will correspond to the position in the list of the provided lines.")]
        public static Output<List<List<PlanarSurface>>, List<List<PlanarSurface>>> VoronoiRegions(List<Point> points = null, List<ICurve> curves = null, int curveDiscretisation = 10, ICurve boundaryCurve = null, List<ICurve> openingCurves = null, Plane plane = null, double boundarySize = -1, double tolerance = Tolerance.Distance, double simplifyDistTol = Tolerance.MacroDistance, double simplifyAngTol = Math.PI / 180 * 3)
        {
            openingCurves = openingCurves ?? new List<ICurve>();
            List<Point> checkingPoints = new List<Point>();
            if (boundaryCurve != null)
                checkingPoints.AddRange(boundaryCurve.IControlPoints());

            checkingPoints.AddRange(openingCurves.SelectMany(x => x.IControlPoints()));

            if (!checkingPoints.IsCoplanar(tolerance))
            {
                Engine.Base.Compute.RecordError("The points, boundaryCurve and openingCurves all need to be coplanar.");
                return new Output<List<List<PlanarSurface>>, List<List<PlanarSurface>>>();
            }

            //Fit plane through boundary curve. This is to better support a linear array of points
            if (plane == null && boundaryCurve != null)
            {
                plane = boundaryCurve.IFitPlane(tolerance);
            }
            if (boundarySize < 0 && boundaryCurve != null)
            {
                BoundingBox boundaryBox = boundaryCurve.IBounds();
                boundarySize = Math.Max(boundaryBox.HorizontalHypotenuseLength(), boundaryBox.Height());
            }

            Output<List<Polyline>, List<List<Polyline>>> untrimmedRegions = VoronoiRegions(points, curves, curveDiscretisation, plane, boundarySize, tolerance, simplifyDistTol, simplifyAngTol);

            List<List<PolyCurve>> trimmedCurves = new List<List<PolyCurve>>();

            List<Tuple<ICurve, BoundingBox>> openingsWithBounds = openingCurves.Select(x => new Tuple<ICurve, BoundingBox>(x, x.IBounds())).ToList();

            return new Output<List<List<PlanarSurface>>, List<List<PlanarSurface>>>
            {
                Item1 = untrimmedRegions.Item1.Select(x => x.TrimWithBoundaryAndOpenings(boundaryCurve, openingsWithBounds, tolerance)).ToList(),
                Item2 = untrimmedRegions.Item2.Select(x => x.SelectMany(y => y.TrimWithBoundaryAndOpenings(boundaryCurve, openingsWithBounds, tolerance)).ToList()).ToList(),
            };
        }

        /***************************************************/
        /****      Private Methods                      ****/
        /***************************************************/

        [Description("Checks if any of the curves are intersecting with each other or touching any of the providing points. Takes different actions depending if there is an issue that will affect the voronoi generation or if it can be fixed with small tweaks. Returns true if an intersection that can not be handled is found.")]
        private static bool AnyIntersectingNotEndPoints(this List<ICurve> curves, List<Point> points, out List<bool> offsetStart, out List<bool> offsetEnd, out List<List<Point>> mustIncludePts, double tolerance)
        {
            offsetStart = Enumerable.Repeat(false, curves.Count).ToList();
            offsetEnd = Enumerable.Repeat(false, curves.Count).ToList();
            mustIncludePts = new List<List<Point>>();

            for (int i = 0; i < curves.Count; i++)
                mustIncludePts.Add(new List<Point>());

            double sqTol = tolerance * tolerance;
            for (int i = 0; i < curves.Count; i++)
            {


                ICurve c1 = curves[i];
                Point stPt = c1.IStartPoint();
                Point enPt = c1.IEndPoint();
                for (int j = 0; j < points.Count; j++)
                {
                    Point closePt = c1.IClosestPoint(points[j]);
                    if (closePt.SquareDistance(points[j]) < sqTol)
                    {
                        if (closePt.SquareDistance(stPt) < sqTol)
                            offsetStart[i] = true;
                        else if (closePt.SquareDistance(enPt) < sqTol)
                            offsetEnd[i] = true;
                        else
                        {
                            Vector v = c1.ITangentAtPoint(closePt);
                            mustIncludePts[i].Add(closePt - v * tolerance * 5);
                            mustIncludePts[i].Add(closePt + v * tolerance * 5);
                        }

                    }
                }

                for (int j = i + 1; j < curves.Count; j++)
                {
                    ICurve c2 = curves[j];

                    List<Point> interPts = c1.ICurveIntersections(c2, tolerance);
                    if (interPts.Count != 0)
                    {
                        foreach (Point point in interPts)
                        {
                            bool endPt = false;
                            if (point.SquareDistance(stPt) < sqTol)
                                offsetStart[i] = endPt = true;
                            else if (point.SquareDistance(enPt) < sqTol)
                                offsetEnd[i] = endPt = true;
                            else
                                mustIncludePts[i].Add(point);

                            if (point.SquareDistance(c2.IStartPoint()) < sqTol)
                                offsetStart[j] = endPt = true;
                            else if (point.SquareDistance(c2.IEndPoint()) < sqTol)
                                offsetEnd[j] = endPt = true;
                            else
                                mustIncludePts[j].Add(point);
                          
                            if(!endPt)
                                return true;
                        }
                    }
                }
            }
            return false;
        }

        [Description("Simplified naive boolean union that joins consecutive closed polylines that share an identical edge by removing those identical edges and then joining the remaining parts.")]
        private static List<Polyline> SimpleBooleanUnion(this List<Polyline> pLines, double tol = Tolerance.Distance)
        {
            List<Line> subParts = pLines.SelectMany(x => x.SubParts()).ToList();//Get all line segments

            List<Line> uniqueLines;
            //Group edges by midpoint
            //if (tol > 1e-7)
            //{
            //    //if tolerance is not to small to risk int overflow, group by use of discrete points rounded up by the tolerance value
            //    Dictionary<DiscretePoint, List<Line>> groupedLines = new Dictionary<DiscretePoint, List<Line>>();
            //    double factor = 1 / (2 * tol);
            //    foreach (Line line in subParts)
            //    {
            //        DiscretePoint pt = new DiscretePoint
            //        {
            //            X = (int)Math.Floor((line.Start.X + line.End.X) * factor),
            //            Y = (int)Math.Floor((line.Start.Y + line.End.Y) * factor),
            //            Z = (int)Math.Floor((line.Start.Z + line.End.Z) * factor)
            //        };

            //        if (groupedLines.ContainsKey(pt))
            //            groupedLines[pt].Add(line);
            //        else
            //            groupedLines[pt] = new List<Line> { line };
            //    }

            //    uniqueLines = groupedLines.Values.Where(x => x.Count < 2).SelectMany(x => x).ToList();


            //}
            //else
            {
                //Else if to small tolerance, use domain box to cluster. SLower but more resiliant to really tight tolerances
                double sqTol = tol * tol;
                Func<Line, DomainBox> toDomainBox = li => new DomainBox()
                {
                    Domains = new Domain[] {
                        new Domain((li.Start.X + li.End.X)/2, (li.Start.X + li.End.X)/2),
                        new Domain((li.Start.Y + li.End.Y)/2, (li.Start.Y + li.End.Y)/2),
                        new Domain((li.Start.Z + li.End.Z)/2, (li.Start.Z + li.End.Z)/2),
                    }
                };
                Func<DomainBox, DomainBox, bool> treeFunction = (a, b) => a.SquareDistance(b) < sqTol;
                Func<Line, Line, bool> itemFunction = (a, b) => true;  // The distance between the boxes is enough to determine if a Point is in range
                List<List<Line>> clusterLines = Data.Compute.DomainTreeClusters<Line>(subParts, toDomainBox, treeFunction, itemFunction, 1);

                //Get out lines that where in a group with only one edge -> unique lines
                uniqueLines = clusterLines.Where(x => x.Count < 2).SelectMany(x => x).ToList();
            }

            return Engine.Geometry.Compute.Join(uniqueLines);
        }

        private static List<PlanarSurface> TrimWithBoundaryAndOpenings(this Polyline pLine, ICurve boundaryCurve, List<Tuple<ICurve, BoundingBox>> openingCurves, double tolerance)
        {
            List<ICurve> boundaryTrimmedCurves;
            if (boundaryCurve != null)
                boundaryTrimmedCurves = pLine.BooleanIntersection(boundaryCurve, tolerance).ToList<ICurve>();
            else
                boundaryTrimmedCurves = new List<ICurve>() { pLine };

            List<PlanarSurface> trimmedRegions = new List<PlanarSurface>();

            foreach (ICurve curve in boundaryTrimmedCurves)
            {
                BoundingBox curveBox = curve.IBounds();
                //Find opening curves in range of the curve to trim
                IEnumerable<ICurve> inRangeOpeningCurves = openingCurves.Where(x => x.Item2.IsInRange(curveBox)).Select(x => x.Item1);

                if (inRangeOpeningCurves.Any()) //If any opening is in range, then use it to trim
                {
                    List<ICurve> openingTrimmedCurves = curve.BooleanDifference(inRangeOpeningCurves, tolerance).ToList<ICurve>();
                    if (openingTrimmedCurves.Any())
                        trimmedRegions.AddRange(Create.PlanarSurface(openingTrimmedCurves, tolerance));
                }
                else   //If no opening in range, skip trimming and add the full curve
                    trimmedRegions.Add(new PlanarSurface(new BoundaryCurve(curve.ISubParts()), new List<ICurve>()));    //Safe to directly call constructor as curve has been checked already by method(s) called.
            }
            return trimmedRegions;
        }

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
                BH.Engine.Base.Compute.RecordError("Can only handle coplanar points, in the plane.");
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





