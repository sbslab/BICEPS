within BICEPS.Fluid.Examples;
model SeriesConstantFlow
  "Example of series connection with constant district water mass flow rate"
  extends Buildings.Experimental.DHC.Examples.Combined.Generation5.Examples.SeriesConstantFlow;
  annotation (
    Icon(coordinateSystem(preserveAspectRatio=false)),
    Diagram(coordinateSystem(preserveAspectRatio=false)),
    __Dymola_Commands(
      file="modelica://BICEPS/Resources/Scripts/Dymola/Fluid/Examples/SeriesConstantFlow.mos"
      "Simulate and plot"),
    experiment(
      StopTime=604800,
      Tolerance=1e-06),
    Documentation(info="<html>
<p>
This is a model of a so-called \"reservoir network\" (Sommer 2020), i.e., a fifth
generation district system with unidirectional mass flow rate in the
district loop, and energy transfer stations connected in series.
In this model, the temperature of the district loop is stabilized through
the operation of the plant and the borefield.
The main circulation pump has a constant mass flow rate.
Each substation takes water from the main district loop and feeds its return water back
into the main district loop downstream from the intake.
The pipes of the main loop are designed for a pressure drop of
<code>dpDis_length_nominal=250</code> Pa/m at the design flow rate.
</p>
<h4>References</h4>
<p>
Sommer T., Sulzer M., Wetter M., Sotnikov A., Mennel S., Stettler C.
<i>The reservoir network: A new network topology for district heating
and cooling.</i>
Energy, Volume 199, 15 May 2020, 117418.
</p>
</html>", revisions="<html>
<ul>
<li>
February 23, 2021, by Antoine Gautier:<br/>
Refactored with base classes from the <code>DHC</code> package.<br/>
This is for
<a href=\"https://github.com/lbl-srg/modelica-buildings/issues/1769\">
issue 1769</a>.
</li>
<li>
January 12, 2020, by Michael Wetter:<br/>
Added documentation.
</li>
</ul>
</html>"));
end SeriesConstantFlow;
