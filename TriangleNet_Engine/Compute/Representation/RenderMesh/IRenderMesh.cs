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
using BH.oM.Graphics;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using BH.oM.Base;
using BH.Engine.Graphics;
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        // Main interface method
        public static BH.oM.Graphics.RenderMesh IRenderMesh(this IObject obj, RenderMeshOptions renderMeshOptions = null)
        {
            if (obj == null) return null;

            renderMeshOptions = renderMeshOptions ?? new RenderMeshOptions();

            RenderMesh renderMesh = null;

            // See if there is a custom BHoM mesh representation for this BHoMObject, before attempting the RenderMesh computation.
            if (obj is IBHoMObject)
                if (TryGetRendermesh(obj as IBHoMObject, out renderMesh, renderMeshOptions.CustomRendermeshKey))
                    return renderMesh;

            if (obj is BH.oM.Graphics.RenderMesh)
                return obj as BH.oM.Graphics.RenderMesh;

            BH.oM.Geometry.Mesh mesh = obj as BH.oM.Geometry.Mesh;
            if (mesh != null)
                return (RenderMesh)mesh;

            IGeometry geomRepr = IGeometricalRepresentation(obj, renderMeshOptions.RepresentationOptions);

            if (geomRepr == null)
                return null; // Could not compute the geometrical representation for that object.

            renderMesh = RenderMesh(geomRepr as dynamic, renderMeshOptions);

            if (renderMesh == null)
                BH.Engine.Reflection.Compute.RecordError($"Could not find a method to compute the {nameof(BH.oM.Graphics.RenderMesh)} of {obj.GetType().Name}");

            return renderMesh;
        }

        // Fallback
        private static BH.oM.Graphics.RenderMesh RenderMesh(this IGeometry geom, RenderMeshOptions renderMeshOptions = null)
        {
            return null;
        }

        [Description("Retrieves a representation from the specified IBHoMObject CustomData key, if present. Returns it as a RenderMesh.\n" +
            "If the representation is stored as a BH.oM.Geometry.Mesh type, it's converted to a RenderMesh.\n" +
            "If the CustomData contains a list of RenderMeshes or Meshes, they are joined together into a single RenderMesh.")]
        public static bool TryGetRendermesh(this IBHoMObject bHoMObject, out RenderMesh renderMesh, string customDataKey = "RenderMesh")
        {
            if (string.IsNullOrWhiteSpace(customDataKey))
            {
                renderMesh = null;
                return false;
            }

            renderMesh = null;

            if (bHoMObject != null)
            {
                object renderMeshObj = null;
                bHoMObject.CustomData.TryGetValue(customDataKey, out renderMeshObj);

                if (renderMeshObj != null)
                {
                    renderMesh = renderMeshObj as RenderMesh;
                    Mesh geomMesh = renderMeshObj as Mesh;

                    if (renderMesh != null)
                        return true;

                    if (geomMesh != null)
                    {
                        renderMesh = geomMesh.ToRenderMesh();
                        return true;
                    }

                    if (typeof(IEnumerable<object>).IsAssignableFrom(renderMeshObj.GetType()))
                    {
                        List<object> objects = renderMeshObj as List<object>;

                        List<RenderMesh> renderMeshes = objects.OfType<RenderMesh>().ToList();
                        List<Mesh> geomMeshes = objects.OfType<Mesh>().ToList();

                        if (geomMeshes.Count > 0)
                        {
                            geomMesh = JoinMeshes(geomMeshes);
                            renderMeshes.Add(geomMesh.ToRenderMesh());
                        }

                        if (renderMeshes.Count > 0)
                            renderMesh = JoinRenderMeshes(renderMeshes);
                    }

                    if (renderMesh != null)
                        return true;
                    else
                        return false;
                }
            }

            return false;
        }

    }
}