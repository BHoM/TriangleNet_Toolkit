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
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text.RegularExpressions;
using BH.Engine.Geometry;
using BH.oM.Base;
using BH.oM.Structure.Elements;
using BH.oM.Structure.Constraints;
using System.ComponentModel;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        /***************************************************/
        /**** Public Methods - Graphics                 ****/
        /***************************************************/

        [Description("Returns the geometrical representation of the Node. It can be as simple as a Sphere, or a more complicated Composite Geometry computed from its DOFs.")]
        public static IGeometry GeometricalRepresentation(this Node node, RepresentationOptions reprOptions = null, bool isSubObject = false)
        {
            if(node == null)
            {
                BH.Engine.Base.Compute.RecordError("Cannot compute the geometrical representation of a null node.");
                return null;
            }

            reprOptions = reprOptions ?? new RepresentationOptions();

            if (node.Position == null)
            {
                BH.Engine.Base.Compute.RecordError("Specified Node does not have a position defined.");
                return null;
            }

            if (node.Support == null || !reprOptions.Detailed0DElements) // If there is no support information, or by choice...
                return node.Position; // ...just return the representation for the point.

            // -------------------------------------------- //
            // -------- Compute the representation -------- //
            // -------------------------------------------- //

            // Design different representations for different DOF type.
            
            // For now, all the following DOFTypes are considered "fixed". More, differentiated representations can be added later
            DOFType[] fixedDOFTypes = new[] { DOFType.Fixed, DOFType.FixedNegative, DOFType.FixedPositive, DOFType.Spring, DOFType.Friction, DOFType.Damped, DOFType.SpringPositive, DOFType.SpringNegative};

            bool fixedToTranslation = fixedDOFTypes.Contains(node.Support.TranslationX) || fixedDOFTypes.Contains(node.Support.TranslationY) || fixedDOFTypes.Contains(node.Support.TranslationZ);
            bool fixedToRotation = fixedDOFTypes.Contains(node.Support.RotationX) || fixedDOFTypes.Contains(node.Support.RotationY) || fixedDOFTypes.Contains(node.Support.RotationZ);

            if (fixedToTranslation && fixedToRotation)
            {
                // Fully fixed: box
                double boxDims = 0.2 * reprOptions.Element0DScale;

                var centrePoint = node.Position;
                BoundingBox bbox = BH.Engine.Geometry.Create.BoundingBox(
                    new Point() { X = centrePoint.X + 2 * boxDims, Y = centrePoint.Y + 2 * boxDims, Z = centrePoint.Z + boxDims * 1.5},
                    new Point() { X = centrePoint.X - 2 * boxDims, Y = centrePoint.Y - 2 * boxDims, Z = centrePoint.Z - boxDims * 1.5 });

                return bbox;
            }

            if (fixedToTranslation && !fixedToRotation)
            {
                // Pin: cone + sphere
                double radius = 0.15 * reprOptions.Element0DScale;

                CompositeGeometry compositeGeometry = new CompositeGeometry();

                Sphere sphere = BH.Engine.Geometry.Create.Sphere(node.Position, radius);
                compositeGeometry.Elements.Add(sphere);

                double coneHeight = 4 * radius;

                Cone cone = BH.Engine.Geometry.Create.Cone(
                    new Point() { X = node.Position.X, Y = node.Position.Y, Z = node.Position.Z - radius/2 - coneHeight },
                    new Vector() { X = 0, Y = 0, Z = 1 },
                    3 * radius,
                    coneHeight
                    );
                compositeGeometry.Elements.Add(cone);

                return compositeGeometry;
            }

            // Else: we could add more for other DOFs; for now just return the representation for its point.
            if (!isSubObject)
                return node.Position.GeometricalRepresentation(reprOptions);
            else
                return null; //do not return representation for point if the Nodes are sub-objects (e.g. of a bar)
        }

    }
}




