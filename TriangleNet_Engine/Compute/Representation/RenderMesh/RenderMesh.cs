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
using BH.oM.Graphics;
using System;
using System.Drawing;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using BH.oM.Base;
using System.ComponentModel;
using BH.oM.Base.Attributes;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        [Description("Returns the RenderMesh representation of an input RenderGeometry.")]
        [Input("renderGeometry", "The RenderGeometry to get a RenderMesh from.")]
        [Input("renderMeshOptions", "Options that regulate both the calculation of the Geometrical Representation and how it should be meshed.")]
        [Output("A RenderMesh, which is a geometrical mesh that can potentially have additional attributes like Colours.")]

        public static BH.oM.Graphics.RenderMesh RenderMesh(this RenderGeometry renderGeometry, RenderMeshOptions renderMeshOptions = null)
        {
            if (renderGeometry == null)
            {
                BH.Engine.Base.Compute.RecordError($"Cannot compute the mesh of a null {nameof(RenderGeometry)} object.");
                return null;
            }

            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            RenderMesh renderMesh = IRenderMesh(renderGeometry.Geometry);

            foreach (RenderPoint vertice in renderMesh.Vertices)
            {
                vertice.Colour = renderGeometry.Colour;
            }

            return renderMesh;
        }
    }
}




