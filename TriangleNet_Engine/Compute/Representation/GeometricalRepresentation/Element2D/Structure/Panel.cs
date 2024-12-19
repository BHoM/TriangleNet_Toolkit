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
using System.ComponentModel;
using BH.oM.Structure.Elements;
using BH.Engine.Structure;
using BH.Engine.Spatial;
using BH.oM.Structure.Constraints;

namespace BH.Engine.Representation
{
    public static partial class Compute
    {
        [Description("Returns the geometrical representation of the Structure Panel. It can be as simple as its middle Surface, Composite Geometry representing its thickness.")]
        public static IGeometry GeometricalRepresentation(this Panel panel, RepresentationOptions reprOptions = null)
        {
            if (panel == null)
            {
                BH.Engine.Base.Compute.RecordError("Cannot compute the geometrical representation of a null Structural Panel.");
                return null;
            }

            reprOptions = reprOptions ?? new RepresentationOptions();

            PlanarSurface centralPlanarSurface = Engine.Geometry.Create.PlanarSurface(
                    Engine.Geometry.Compute.IJoin(panel.ExternalEdges.Select(x => x.Curve).ToList()).FirstOrDefault(),
                    panel.Openings.SelectMany(x => Engine.Geometry.Compute.IJoin(x.Edges.Select(y => y.Curve).ToList())).Cast<ICurve>().ToList());

            if (!reprOptions.Detailed2DElements) //simple representation
                return centralPlanarSurface;
            else
            {
                CompositeGeometry compositeGeometry = new CompositeGeometry();

                double thickness = panel.Property.ITotalThickness();
                Vector translateVect = new Vector() { Z = -thickness / 2 };
                Vector extrudeVect = new Vector() { Z = thickness };

                Vector upHalf = new Vector() { X = 0, Y = 0, Z = thickness / 2 };
                Vector downHalf = new Vector() { X = 0, Y = 0, Z = -thickness / 2 };

                PlanarSurface topSrf = centralPlanarSurface.ITranslate(upHalf) as PlanarSurface;
                PlanarSurface botSrf = centralPlanarSurface.ITranslate(downHalf) as PlanarSurface;

                IEnumerable<ICurve> internalEdgesBot = panel.InternalElementCurves().Select(c => c.ITranslate(translateVect));
                IEnumerable<Extrusion> internalEdgesExtrusions = internalEdgesBot.Select(c => BH.Engine.Geometry.Create.Extrusion(c, extrudeVect));

                IEnumerable<ICurve> externalEdgesBot = panel.ExternalEdges.Select(c => c.Curve.ITranslate(translateVect));
                IEnumerable<Extrusion> externalEdgesExtrusions = externalEdgesBot.Select(c => BH.Engine.Geometry.Create.Extrusion(c, extrudeVect));

                compositeGeometry.Elements.Add(topSrf);
                compositeGeometry.Elements.Add(botSrf);
                compositeGeometry.Elements.AddRange(internalEdgesExtrusions);
                compositeGeometry.Elements.AddRange(externalEdgesExtrusions);

                return compositeGeometry;
            }
        }
    }
}




