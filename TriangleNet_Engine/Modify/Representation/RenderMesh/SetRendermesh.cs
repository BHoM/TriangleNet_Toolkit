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
        [Description("Sets a RenderMesh in the Fragments of the BHoMObject.\n" +
           "If specified, the `meshRepresentation` should contain a valid mesh representation for the BHoMObject." +
           "If meshRepresentation is null, attempts to compute a Rendermesh for the object. ")]
        [Input("bHoMObject", "BHoMObject to set the representation for.")]
        [Input("meshRepresentation", "A valid mesh representation for the BHoMObject.")]
        public static IBHoMObject ISetRendermesh(this IBHoMObject bHoMObject, object meshRepresentation = null, RenderMeshOptions renderMeshOptions = null)
        {
            if (bHoMObject == null)
                return null;

            if (meshRepresentation == null)
            {
                // Try computing a RenderMesh from the specified representation Geometry
                RenderMesh rm = Compute.IRenderMesh(bHoMObject, renderMeshOptions);

                if (rm == null)
                {
                    BH.Engine.Reflection.Compute.RecordError($"The input {nameof(meshRepresentation)} was null. The method attempted to compute a RenderMesh for the object, but this failed.");
                    return null;
                }

                return SetRendermesh(bHoMObject, rm);
            }

            return SetRendermesh(bHoMObject, meshRepresentation as dynamic);
        }

        private static IBHoMObject SetRendermesh(this IBHoMObject bHoMObject, RenderMesh renderMesh)
        {
            bHoMObject.Fragments.AddOrReplace(renderMesh);

            return bHoMObject;
        }

        private static IBHoMObject SetRendermesh(this IBHoMObject bHoMObject, Mesh representation)
        {
            RenderMesh rm = representation.ToRenderMesh();
            return bHoMObject.SetRendermesh(rm);
        }

        private static IBHoMObject SetRendermesh(this IBHoMObject bHoMObject, List<Mesh> representation)
        {
            Mesh m = Compute.JoinMeshes(representation);
            RenderMesh rm = m.ToRenderMesh();
            return bHoMObject.SetRendermesh(m);
        }

        private static IBHoMObject SetRendermesh(this IBHoMObject bHoMObject, List<RenderMesh> representation)
        {
            RenderMesh m = Compute.JoinRenderMeshes(representation);
            return bHoMObject.SetRendermesh(m);
        }

        // Fallback
        private static IBHoMObject SetRendermesh(this IBHoMObject bHoMObject, object representation)
        {
            BH.Engine.Reflection.Compute.RecordError("Invalid input used for the representation.");
            return null;
        }
    }
}


