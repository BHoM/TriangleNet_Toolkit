/*
 * This file is part of the Buildings and Habitats object Model (BHoM)
 * Copyright (c) 2015 - 2023, the respective contributors. All rights reserved.
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
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        [Description("Joins multiple Meshes into a single one. Currently this does not optimise for duplicate vertices.")]
        public static Mesh JoinMeshes(List<Mesh> meshes)
        {
            meshes = meshes.Where(m => m != null).ToList();
            if (meshes.Count < 2)
                return meshes.FirstOrDefault();

            List<BH.oM.Geometry.Point> vertices = new List<BH.oM.Geometry.Point>();
            List<Face> faces = new List<Face>();

            vertices.AddRange(meshes[0].Vertices);
            faces.AddRange(meshes[0].Faces);

            for (int i = 1; i < meshes.Count; i++)
            {
                int lastVerticesCount = vertices.Count;
                vertices.AddRange(meshes[i].Vertices);
                faces.AddRange(
                    meshes[i].Faces.Select(f =>
                        new Face() { A = f.A + lastVerticesCount, B = f.B + lastVerticesCount, C = f.C + lastVerticesCount, D = f.D == -1 ? f.D : f.D + lastVerticesCount }));
            }

            return new Mesh() { Vertices = vertices, Faces = faces };
        }

        /***************************************************/
    }
}




