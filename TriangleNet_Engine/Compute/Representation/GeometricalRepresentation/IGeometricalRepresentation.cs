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
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        [Description("Computes the Geometrical Representation of the given IObject." +
            "The Geometrical Representation is an IGeometry that can be used to represent the object in another environment.")] //e.g. to 3D-print a point, you want to print a Sphere.
        public static IGeometry IGeometricalRepresentation(this IObject obj, RepresentationOptions reprOptions = null)
        {
            reprOptions = reprOptions ?? new RepresentationOptions();

            IGeometry geometricalRepresentation = null;

            // First check if there is a specific GeometricalRepresentation method for the IObject.
            geometricalRepresentation = GeometricalRepresentation(obj as dynamic, reprOptions);
            if (geometricalRepresentation != null)
                return geometricalRepresentation;

            // If not, check if the object is a IGeometry, and if it is return itself.
            geometricalRepresentation = obj as IGeometry;
            if (geometricalRepresentation != null)
                return geometricalRepresentation;

            // If not, check if the object is a IBHoMObject whose IGeometry can be returned.
            IBHoMObject bHoMObject = obj as IBHoMObject;
            if (bHoMObject != null)
                geometricalRepresentation = BH.Engine.Base.Query.IGeometry(bHoMObject);

            if (geometricalRepresentation == null)
                BH.Engine.Reflection.Compute.RecordError($"Could not compute the Geometrical Representation for the object of type {obj.GetType().Name}");

            return geometricalRepresentation;
        }

        // Fallback
        private static IGeometry GeometricalRepresentation(this IObject obj, RepresentationOptions reprOptions = null)
        {
            return null;
        }
    }
}