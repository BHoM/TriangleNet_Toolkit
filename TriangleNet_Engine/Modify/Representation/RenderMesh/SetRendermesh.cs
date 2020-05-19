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


using BH.oM.Base;
using BH.oM.Geometry;
using BH.oM.Graphics;
using BH.Engine.Graphics;
using BH.oM.Reflection.Attributes;
using System;
using System.Collections.Generic;
using System.Linq;
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Modify
    {
        [Description("Sets a RenderMesh in the CustomData of the BHoMObject at the specified customDataKey (default: `RenderMesh`).\n" +
            "The RenderMesh should contain a valid representation for the BHoMObject.")]
        public static IBHoMObject SetRendermesh(this IBHoMObject bHoMObject, IGeometry representation, string customDataKey = "RenderMesh")
        {
            if (bHoMObject == null || representation == null || string.IsNullOrWhiteSpace(customDataKey))
                return null;

            RenderMesh rM = representation as RenderMesh;
            Mesh m = representation as Mesh;

            if (rM == null && m != null)
                rM = m.ToRenderMesh();

            if (rM == null)
            {
                // Try computing a RenderMesh from the specified representation Geometry
                BH.Engine.Reflection.Compute.RecordNote($"The input {nameof(representation)} is neither a {nameof(BH.oM.Graphics.RenderMesh)} or a {nameof(BH.oM.Geometry.Mesh)}. Attempting to compute a RenderMesh for it.");

                rM = Compute.IRenderMesh(representation);
            }

            IBHoMObject clonedObj = bHoMObject.GetShallowClone();

            clonedObj.CustomData[customDataKey] = rM;

            return clonedObj;
        }
    }
}

