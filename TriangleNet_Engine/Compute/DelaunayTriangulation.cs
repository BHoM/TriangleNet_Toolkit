/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2021, the respective contributors. All rights reserved.
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
using BH.oM.Quantities.Attributes;
using BH.Engine.Geometry;

using System.ComponentModel;

namespace BH.Engine.Geometry.Triangulation
{
    public static partial class Compute
    {
        /***************************************************/
        /****      public Methods                       ****/
        /***************************************************/

        [Description("Create a Delaunay mesh from an outline and holes")]
        [Input("outerCurve", "A BHoM Polyline representing the mesh boundary")]
        [Input("innerCurves", "A list of holes to \"punch\" through the mesh generated mesh")]
        [Input("offsetDistance", "Offset distance for innerCurves which have coincident edges - needs to be a negative value for inwards based offsetting, default -0.001", typeof(Length))]
        [Input("conformingDelaunay", "Choose whether or not to have the resulting triangulation conform to Delaunay principles. This will give a higher detail triangulation. Default true")]
        [Output("curve", "A list of BHoM Polylines")]
        public static List<Polyline> DelaunayTriangulation(this Polyline outerCurve, List<Polyline> innerCurves = null, double offsetDistance = -0.001, bool conformingDelaunay = true)
        {
            if (outerCurve == null)
            {
                BH.Engine.Reflection.Compute.RecordError("Cannot perform Delaunay Triangulation on an outer curve that is set to null.");
                return new List<Polyline>();
            }

            // Create a zero length list if no holes input
            if (innerCurves == null)
                innerCurves = new List<Polyline>();

            double area = outerCurve.Area();
            for (int x = 0; x < innerCurves.Count; x++)
            {
                Polyline pLine = outerCurve.BooleanDifference(new List<Polyline> { innerCurves[x] })[0];

                if (pLine.Area() != area)
                {
                    //The boolean difference returned a different polyline - offset this inner curve
                    innerCurves[x] = innerCurves[x].Offset(offsetDistance);
                }
            }

            // Get the transformation matrix
            Plane plane = outerCurve.IFitPlane();
            Vector normal = plane.Normal;
            List<Point> vertices = outerCurve.IDiscontinuityPoints();
            Point refPoint = vertices.Min();
            Point refPointP = BH.Engine.Geometry.Create.Point(refPoint.X, refPoint.Y, 0);
            Vector zVector = BH.Engine.Geometry.Create.Vector(0, 0, 1);
            Vector rotationVector = normal.CrossProduct(zVector).Normalise();
            double rotationAngle = normal.Angle(zVector);
            TransformMatrix transformMatrix = BH.Engine.Geometry.Create.RotationMatrix(vertices.Min(), rotationVector, rotationAngle);

            // Get the translation vector
            Vector translateVector = refPointP - refPoint;

            // Transform the original input curve/s
            Polyline transformedCurve = Modify.Translate(outerCurve.Transform(transformMatrix), translateVector);
            List<Polyline> transformedHole = new List<Polyline>();
            foreach (Polyline h in innerCurves)
            {
                if (h.IsCoplanar(outerCurve))
                {
                    transformedHole.Add(Modify.Translate(h.Transform(transformMatrix), translateVector));
                }
            }

            // Convert geometry to Triangle inputs
            TriangleNet.Geometry.Polygon parentPolygon = new TriangleNet.Geometry.Polygon();
            List<TriangleNet.Geometry.Vertex> parentVertices = new List<TriangleNet.Geometry.Vertex>();
            foreach (Point point in transformedCurve.IDiscontinuityPoints())
            {
                parentPolygon.Add(new TriangleNet.Geometry.Vertex(point.X, point.Y));
                parentVertices.Add(new TriangleNet.Geometry.Vertex(point.X, point.Y));
            }
            TriangleNet.Geometry.Contour parentContour = new TriangleNet.Geometry.Contour(parentVertices);
            parentPolygon.Add(parentContour);

            foreach (Polyline h in transformedHole)
            {
                List<TriangleNet.Geometry.Vertex> childVertices = new List<TriangleNet.Geometry.Vertex>();
                foreach (Point point in h.IDiscontinuityPoints())
                {
                    childVertices.Add(new TriangleNet.Geometry.Vertex(point.X, point.Y));
                }
                TriangleNet.Geometry.Contour childContour = new TriangleNet.Geometry.Contour(childVertices);
                Point childCentroid = h.PointInRegion();
                parentPolygon.Add(childContour, new TriangleNet.Geometry.Point(childCentroid.X, childCentroid.Y));
            }

            // Triangulate
            TriangleNet.Meshing.ConstraintOptions options = new TriangleNet.Meshing.ConstraintOptions() { ConformingDelaunay = conformingDelaunay, };
            TriangleNet.Meshing.QualityOptions quality = new TriangleNet.Meshing.QualityOptions() { };
            TriangleNet.Mesh mesh = (TriangleNet.Mesh)TriangleNet.Geometry.ExtensionMethods.Triangulate(parentPolygon, options, quality);

            // Convert triangulations back to BHoM geometry
            List<Polyline> translatedPolylines = new List<Polyline>();
            foreach (var face in mesh.Triangles)
            {
                // List points defining the triangle
                List<Point> pts = new List<Point>();
                pts.Add(BH.Engine.Geometry.Create.Point(face.GetVertex(0).X, face.GetVertex(0).Y));
                pts.Add(BH.Engine.Geometry.Create.Point(face.GetVertex(1).X, face.GetVertex(1).Y));
                pts.Add(BH.Engine.Geometry.Create.Point(face.GetVertex(2).X, face.GetVertex(2).Y));
                pts.Add(pts.First());
                translatedPolylines.Add(BH.Engine.Geometry.Create.Polyline(pts));
            }

            // Translate back to original plane
            List<Polyline> meshPolylines = new List<Polyline>();
            foreach (Polyline pl in translatedPolylines)
            {
                TransformMatrix matrixTransposed = transformMatrix.Invert();
                Polyline meshPolyline = pl.Translate(-translateVector).Transform(transformMatrix.Invert());
                meshPolylines.Add(meshPolyline);
            }

            return meshPolylines;
        }

        /***************************************************/

        [Description("Creates a list of closed Delaunay polyline cells from a list of coplanar points. Each cell will correspond to a face in a Delaunay mesh.")]
        [Input("points", "The points to connect to Delanuay cells. All points need to be coplanar ")]
        [Input("plane", "Optional plane of the points. Needs to be coplanar with all of the points. If no plane is provided, a best fit plane through the points will be used.")]
        [Input("tolerance", "Numeric tolerance to be used by the method.", typeof(Length))]
        [Output("curves", "List of closed polyline representing faces in a Delaunay.")]
        public static List<Polyline> DelaunayTriangulation(List<Point> points, Plane plane = null, double tolerance = Tolerance.Distance)
        {
            //Preform check all inputs that triangulation can be done
            if (points == null || points.Count < 3)
            {
                Reflection.Compute.RecordError("Insufficient points for triangulation. Please provide at least 3 Points.");
                return new List<Polyline>();
            }

            if (points.IsCollinear(tolerance))
            {
                Reflection.Compute.RecordError("Points are colinear and can not be triangulated.");
                return new List<Polyline>();
            }

            //Basic edge case, simply create a triangle
            if (points.Count == 3)
            {
                Polyline pLine = new Polyline { ControlPoints = points };
                pLine.ControlPoints.Add(points[0]);
                return new List<Polyline> { pLine };
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

            //Convert to TringleNet vertecies
            List<TriangleNet.Geometry.Vertex> vertecies = xyPoints.Select(p => new TriangleNet.Geometry.Vertex(p.X, p.Y)).ToList();

            // Triangulate
            TriangleNet.Meshing.ITriangulator triangulator = new TriangleNet.Meshing.Algorithm.Dwyer();
            TriangleNet.Configuration config = new TriangleNet.Configuration();
            TriangleNet.Mesh mesh = (TriangleNet.Mesh)triangulator.Triangulate(vertecies, config);

            // Convert triangulations back to BHoM geometry
            List<Polyline> translatedPolylines = new List<Polyline>();
            foreach (var face in mesh.Triangles)
            {
                // List points defining the triangle
                List<Point> pts = new List<Point>();
                pts.Add(BH.Engine.Geometry.Create.Point(face.GetVertex(0).X, face.GetVertex(0).Y));
                pts.Add(BH.Engine.Geometry.Create.Point(face.GetVertex(1).X, face.GetVertex(1).Y));
                pts.Add(BH.Engine.Geometry.Create.Point(face.GetVertex(2).X, face.GetVertex(2).Y));
                pts.Add(pts.First());
                translatedPolylines.Add(BH.Engine.Geometry.Create.Polyline(pts));
            }

            return translatedPolylines.Select(x => x.Orient(globalSystem, localSystem)).ToList();
        }


        /***************************************************/
    }
}


