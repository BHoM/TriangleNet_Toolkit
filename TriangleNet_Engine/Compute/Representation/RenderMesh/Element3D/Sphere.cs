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
using BH.oM.Graphics;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using BH.oM.Base;
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        public static BH.oM.Graphics.RenderMesh RenderMesh(this Sphere sphere, RenderMeshOptions renderMeshOptions = null)
        {
            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            BH.Engine.Reflection.Compute.RecordNote("RenderMesh for sphere still doesn't work properly and needs to be finished. A cube is output instead of a sphere.");

            double radius = sphere.Radius;

            // // - Sphere still doesn't work properly, for now just return a little cube instead of a sphere.
            Cuboid cuboid = BH.Engine.Geometry.Create.Cuboid(BH.Engine.Geometry.Create.CartesianCoordinateSystem(sphere.Centre, BH.Engine.Geometry.Create.Vector(1, 0, 0), BH.Engine.Geometry.Create.Vector(0, 1, 0)), radius, radius, radius);
            return cuboid.RenderMesh(renderMeshOptions);

            // // - WIP Code for sphere mesh.

            int nLongitude = 6;                  // Number of vertical lines.
            int nLatitude = nLongitude / 2;      // Number of horizontal lines. A good sphere mesh has about half the number of longitude lines than latitude.

            double DEGS_TO_RAD = Math.PI / 180;
            int numVertices = 0;

            int p, s;
            double x, y, z;
            int nPitch = nLongitude + 1;

            double interLatitudeAngle = (180 / (nLatitude + 1));
            interLatitudeAngle = interLatitudeAngle * DEGS_TO_RAD;
            double interLongitudeAngle = (360 / nLongitude);
            interLongitudeAngle = interLongitudeAngle * DEGS_TO_RAD;
            Point centrePoint = sphere.Centre;

            // ------- Generate all points -------- //

            List<Point> allPoints = new List<Point>();

            Point top = new Point() { X = sphere.Centre.X, Y = sphere.Centre.Y, Z = sphere.Centre.Z + radius };
            allPoints.Add(top);

            for (p = 1; p < nLatitude + 1; p++)     // Generate all "intermediate vertices"
            {
                double pitchAngleFromZ = interLatitudeAngle * p;
                z = centrePoint.Z + radius * Math.Cos(pitchAngleFromZ);

                for (s = 0; s < nLongitude + 1; s++)
                {
                    x = centrePoint.X + radius * Math.Cos(s * interLongitudeAngle);
                    y = centrePoint.Y + radius * Math.Sin(s * interLongitudeAngle);

                    allPoints.Add(new Point() { X = x, Y = y, Z = z });
                }
            }

            Point bottom = new Point() { X = sphere.Centre.X, Y = sphere.Centre.Y, Z = sphere.Centre.Z - radius};
            allPoints.Add(bottom);

            // ------- Generate all faces -------- //

            List<Face> allFaces = new List<Face>();

            // Square faces between intermediate points
            for (int lat = 1; lat < nLatitude; lat++)
            {
                for (int lon = 1; lon < nLongitude + 1; lon++)
                {
                    Face face = new Face()
                    {
                        A = lon * lat + nLongitude * (lat - 1),
                        B = lon * lat + 1 + nLongitude * (lat - 1),
                        C = lon * lat + nLongitude + nLongitude * (lat - 1) + 2,
                        D = lon * lat + nLongitude + nLongitude * (lat - 1) + 1,
                    };
                    allFaces.Add(face);
                }
            }

            //// Triangle faces between top/bottom points and the intermediate points
            //int offLastVerts = 2 + (nLatitude * (nLongitude - 1));
            //for (s = 0; s < nLatitude; s++)
            //{
            //    j = (s == nLatitude - 1) ? -1 : s;
            //    allFaces.Add(new Face() { A = 0, B = (j + 2) + 2, C = (s + 1) + 2 });
            //    allFaces.Add(new Face() { A = 1, B = (s + 1) + offLastVerts, C = (j + 2) + offLastVerts });
            //}
            //return null;

            return new RenderMesh() { Faces = allFaces, Vertices = allPoints.Select(pt => (Vertex)pt).ToList() };
        }

    }
}
