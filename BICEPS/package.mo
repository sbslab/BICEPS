within ;
package BICEPS "Modelica library for the Biomimetic Integrated Energy and Power System (BICEPS)"
  extends Modelica.Icons.Package;

package UsersGuide "User's Guide"
  extends ModelicaReference.Icons.Information;
end UsersGuide;

  annotation (
 uses(Modelica(version="3.2.3"),Buildings(version="8.0.0"),
    ModelicaServices(version="3.2.3")),
 Icon(graphics={Line(points={{-2,90}}, color={28,108,200}),
  Text(
   extent={{-100,100},{100,-100}},
   lineColor={0,140,72},
   pattern=LinePattern.None,
   fillColor={0,140,72},
   fillPattern=FillPattern.Solid,
   textStyle={TextStyle.Bold},
   textString="B")}),
 Documentation(info="<html>
    <p>
    This package contains models for the Biomimetic Integrated 
    Energy and Power System (BICEPS).
    </p>
    <h4>Contributors</h4>
    <ul>
    <li>
    Kathryn Hinkelman, University of Colorado Boulder <br>
    kathryn.hinkelman@colorado.edu
    </li>
    </ul>        
 </html>"));
end BICEPS;
