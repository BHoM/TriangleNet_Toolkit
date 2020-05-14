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

using System.ComponentModel;

namespace BH.Engine.Geometry.Triangulation
{
    public static partial class Compute
    {
        /***************************************************/
        /****      public Methods                       ****/
        /***************************************************/

        public static List<Polyline> VoronoiRegions(List<Point> points, double tolerance = Tolerance.Distance)
        {
            //Preform check all inputs that triangulation can be done
            if (points == null || points.Count < 3)
            {
                Reflection.Compute.RecordError("Insuffient points for triangulation. Please provide at least 3 Points!");
                return new List<Polyline>();
            }

            if (points.IsCollinear(tolerance))
            {
                Reflection.Compute.RecordError("Points are colinear and can not be triangulated.");
                return new List<Polyline>();
            }

            //Try fit plane if no is provided
            Plane plane = points.FitPlane(tolerance);

            if (plane == null)
            {
                Engine.Reflection.Compute.RecordError("Could not fit a plane through the Points.");
                return new List<Polyline>();
            }

            //Check all points within distance of the plane
            if (points.Any(x => x.Distance(plane) > tolerance))
            {
                BH.Engine.Reflection.Compute.RecordError("Can only handle coplanar points!");
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


            //Add point at all corners. This is to try to handle weirness at the boundaries
            BoundingBox bounds = xyPoints.Bounds();
            double lengthX = (bounds.Max.X - bounds.Min.X);
            double lengthY = (bounds.Max.X - bounds.Min.X);
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

            // Convert regions to BHoMGeometry. Skip the last 4 faces as they correspond to the added boundary points.
            List<Polyline> translatedPolylines = new List<Polyline>();
            for (int i = 0; i < voronoi.Faces.Count-4; i++)
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
                        pts.Add(new Point { X = halfEdge.Origin.X, Y = halfEdge.Origin.Y });
                        halfEdge = halfEdge.Next;

                        if (halfEdge == null)
                            break;

                        nextId = halfEdge.ID;
                        counter++;
                    } while (!visitedIds.Contains(nextId) && counter < bailOut);


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

        public static List<List<PolyCurve>> VoronoiRegions(List<Point> points, ICurve boundaryCurve, List<ICurve> openingCurves, double tolerance = Tolerance.Distance)
        {

            List<Point> checkingPoints = new List<Point>(points);
            checkingPoints.AddRange(boundaryCurve.IControlPoints());
            checkingPoints.AddRange(openingCurves.SelectMany(x => x.IControlPoints()));

            if (!checkingPoints.IsCoplanar(tolerance))
            {
                Engine.Reflection.Compute.RecordError("The points, boundaryCurve and openingCurves all need to be coplanar.");
                return new List<List<PolyCurve>>();
            }

            List<Polyline> untrimmedRegions = VoronoiRegions(points, tolerance);

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
    }
}
