within BICEPS.ThermalZones;
model SimpleRoomFourElements
  "A thermal zone with four heat conduction elements"
  extends Buildings.BaseClasses.BaseIconLow;
  replaceable package Medium=Buildings.Media.Air
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Load side medium";
  parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
  parameter Real k(min=Modelica.Constants.small)=10
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";

  Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil[2](
    each outSkyCon=true,
    each outGroCon=true,
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={3.1415926535898,4.7123889803847})
    "Calculates diffuse solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil[2](
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={3.1415926535898,4.7123889803847})
    "Calculates direct solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-80,52},{-60,72}})));
  Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane corGDouPan(UWin=2.1,
      n=2) "Correction factor for solar transmission"
    annotation (Placement(transformation(extent={{-6,46},{14,66}})));
  Buildings.ThermalZones.ReducedOrder.RC.FourElements thermalZoneFourElements(
    VAir=52.5,
    hConExt=2.7,
    hConWin=2.7,
    gWin=1,
    ratioWinConRad=0.09,
    nExt=1,
    RExt={0.00331421908725},
    CExt={5259932.23},
    hRad=5,
    AInt=60.5,
    hConInt=2.12,
    nInt=1,
    RInt={0.000668895639141},
    CInt={12391363.86},
    RWin=0.01642857143,
    RExtRem=0.1265217391,
    AFloor=11.5,
    hConFloor=2.7,
    nFloor=1,
    RFloor={0.00331421908725},
    RFloorRem=0.1265217391,
    CFloor={5259932.23},
    ARoof=11.5,
    hConRoof=2.7,
    nRoof=1,
    RRoof={0.00331421908725},
    RRoofRem=0.1265217391,
    CRoof={5259932.23},
    nOrientations=2,
    AWin={7,7},
    ATransparent={7,7},
    AExt={3.5,8},
    redeclare replaceable package Medium = Medium,
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    extWallRC(thermCapExt(each der_T(fixed=true))),
    intWallRC(thermCapInt(each der_T(fixed=true))),
    floorRC(thermCapExt(each der_T(fixed=true))),
    T_start=295.15,
    roofRC(thermCapExt(each der_T(fixed=true))),
    nPorts=2)                                    "Thermal zone"
    annotation (Placement(transformation(extent={{32,-2},{80,34}})));
  Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow
    eqAirTemp(
    wfGro=0,
    withLongwave=true,
    aExt=0.7,
    hConWallOut=20,
    hRad=5,
    hConWinOut=20,
    n=2,
    wfWall={0.3043478260869566,0.6956521739130435},
    wfWin={0.5,0.5},
    TGro=285.15) "Computes equivalent air temperature"
    annotation (Placement(transformation(extent={{-36,-14},{-16,6}})));
  Modelica.Blocks.Math.Add solRad[2]
    "Sums up solar radiation of both directions"
    annotation (Placement(transformation(extent={{-50,6},{-40,16}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature preTem
    "Prescribed temperature for exterior walls outdoor surface temperature"
    annotation (Placement(transformation(extent={{-4,-6},{8,6}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature preTem1
    "Prescribed temperature for windows outdoor surface temperature"
    annotation (Placement(transformation(extent={{-4,14},{8,26}})));
  Modelica.Thermal.HeatTransfer.Components.Convection theConWin
    "Outdoor convective heat transfer of windows"
    annotation (Placement(transformation(extent={{26,14},{14,26}})));
  Modelica.Thermal.HeatTransfer.Components.Convection theConWall
    "Outdoor convective heat transfer of walls"
    annotation (Placement(transformation(extent={{26,6},{14,-6}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow perRad
    "Radiative heat flow of persons"
    annotation (Placement(transformation(extent={{0,-42},{20,-22}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow perCon
    "Convective heat flow of persons"
    annotation (Placement(transformation(extent={{0,-60},{20,-40}})));
  Modelica.Blocks.Sources.CombiTimeTable intGai(
    table=[0,0,0,0; 3600,0,0,0; 7200,0,0,0; 10800,0,0,0; 14400,0,0,0; 18000,0,0,
        0; 21600,0,0,0; 25200,0,0,0; 25200,80,80,200; 28800,80,80,200; 32400,80,
        80,200; 36000,80,80,200; 39600,80,80,200; 43200,80,80,200; 46800,80,80,200;
        50400,80,80,200; 54000,80,80,200; 57600,80,80,200; 61200,80,80,200; 61200,
        0,0,0; 64800,0,0,0; 72000,0,0,0; 75600,0,0,0; 79200,0,0,0; 82800,0,0,0;
        86400,0,0,0],
    columns={2,3,4},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic) "Table with profiles for persons (radiative and convective) and machines
    (convective)"
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
  Modelica.Blocks.Sources.Constant const[2](each k=0)
    "Sets sunblind signal to zero (open)"
    annotation (Placement(transformation(extent={{-36,14},{-30,20}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(transformation(extent={{-18,84},{16,116}}),
        iconTransformation(extent={{-10,90},{10,110}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow macConv
    "Convective heat flow of machines"
    annotation (Placement(transformation(extent={{0,-80},{20,-60}})));
  Modelica.Blocks.Sources.Constant hConWall(k=25*11.5)
    "Outdoor coefficient of heat transfer for walls"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90,
    origin={20,-16})));
  Modelica.Blocks.Sources.Constant hConWin(k=20*14)
    "Outdoor coefficient of heat transfer for windows"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},
    rotation=90,origin={20,38})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature preTemFloor
    "Prescribed temperature for floor plate outdoor surface temperature"
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
    rotation=90,origin={57,-12})));
  Modelica.Blocks.Sources.Constant TSoil(k=283.15)
    "Outdoor surface temperature for floor plate"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},
    rotation=180,origin={40,-22})));
  Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007 eqAirTempVDI(
    aExt=0.7,
    n=1,
    wfWall={1},
    wfWin={0},
    wfGro=0,
    hConWallOut=20,
    hRad=5,
    TGro=285.15) "Computes equivalent air temperature for roof"
    annotation (Placement(transformation(extent={{18,74},{38,94}})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature preTemRoof
    "Prescribed temperature for roof outdoor surface temperature"
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},rotation=-90,
    origin={55,64})));
  Modelica.Thermal.HeatTransfer.Components.Convection theConRoof
    "Outdoor convective heat transfer of roof"
    annotation (Placement(transformation(extent={{5,-5},{-5,5}},rotation=-90,
    origin={55,47})));
  Modelica.Blocks.Sources.Constant hConRoof(k=25*11.5)
    "Outdoor coefficient of heat transfer for roof"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},origin={74,47})));
  Modelica.Blocks.Sources.Constant const1(k=0)
    "Sets sunblind signal to zero (open)"
    annotation (Placement(transformation(extent={{56,90},{50,96}})));

  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{-110,-90},{-90,-70}}),
        iconTransformation(extent={{-110,-90},{-90,-70}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare package Medium = Medium)
    annotation (Placement(transformation(extent={{90,-90},{110,-70}}),
        iconTransformation(extent={{90,-90},{110,-70}})));
  Modelica.Blocks.Interfaces.RealOutput y "Control signal"
    annotation (Placement(transformation(extent={{100,70},{120,90}}),
        iconTransformation(
        extent={{100,60},{120,80}})));
  Utilities.Math.CubicHermite spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0,
    final k=k,
    final ensureMonotonicity=true)
    annotation (Placement(transformation(extent={{76,70},{96,90}})));
equation
  connect(eqAirTemp.TEqAirWin, preTem1.T)
    annotation (Line(points={{-15,-0.2},{-12,-0.2},{-12,20},{-5.2,20}},
    color={0,0,127}));
  connect(eqAirTemp.TEqAir, preTem.T)
    annotation (Line(points={{-15,-4},{-8,-4},{-8,0},{-5.2,0}},
    color={0,0,127}));
  connect(weaBus.TDryBul, eqAirTemp.TDryBul)
    annotation (Line(points={{-1,100},{-4,100},{-4,90},{-90,90},{-90,-10},{-38,-10}},
    color={255,204,51},
    thickness=0.5), Text(textString="%first",index=-1,extent={{-6,3},{-6,3}}));
  connect(intGai.y[1], perRad.Q_flow)
    annotation (Line(points={{-19,-50},{-16,-50},{-16,-32},{0,-32}},
    color={0,0,127}));
  connect(intGai.y[2], perCon.Q_flow)
    annotation (Line(points={{-19,-50},{0,-50}},            color={0,0,127}));
  connect(intGai.y[3], macConv.Q_flow)
    annotation (Line(points={{-19,-50},{-16,-50},{-16,-70},{0,-70}},
    color={0,0,127}));
  connect(const.y, eqAirTemp.sunblind)
    annotation (Line(points={{-29.7,17},{-26,17},{-26,8}},
    color={0,0,127}));
  connect(HDifTil.HSkyDifTil, corGDouPan.HSkyDifTil)
    annotation (Line(points={{-59,36},{-18,36},{-18,58},{-12,58},{-12,57.8},{-8,
          57.8},{-8,58}},
    color={0,0,127}));
  connect(HDirTil.H, corGDouPan.HDirTil)
    annotation (Line(points={{-59,62},{-8,62}},        color={0,0,127}));
  connect(HDirTil.H,solRad. u1)
    annotation (Line(points={{-59,62},{-54,62},{-54,14},{-51,14}},
    color={0,0,127}));
  connect(HDifTil.H,solRad. u2)
    annotation (Line(points={{-59,30},{-56,30},{-56,8},{-51,8}},
    color={0,0,127}));
  connect(HDifTil.HGroDifTil, corGDouPan.HGroDifTil)
    annotation (Line(points={{-59,24},{-16,24},{-16,54},{-8,54}},
    color={0,0,127}));
  connect(solRad.y, eqAirTemp.HSol)
    annotation (Line(points={{-39.5,11},{-38,11},{-38,2}},
    color={0,0,127}));
  connect(perRad.port, thermalZoneFourElements.intGainsRad)
    annotation (
    Line(points={{20,-32},{88,-32},{88,24},{80,24}},
    color={191,0,0}));
  connect(theConWin.solid, thermalZoneFourElements.window)
    annotation (Line(points={{26,20},{32,20}},                   color=
    {191,0,0}));
  connect(preTem1.port, theConWin.fluid)
    annotation (Line(points={{8,20},{14,20}},          color={191,0,0}));
  connect(thermalZoneFourElements.extWall, theConWall.solid)
    annotation (Line(points={{32,12},{28,12},{28,0},{26,0}},
    color={191,0,0}));
  connect(theConWall.fluid, preTem.port)
    annotation (Line(points={{14,0},{8,0}},                color={191,0,0}));
  connect(hConWall.y, theConWall.Gc)
    annotation (Line(points={{20,-11.6},{20,-6}},         color={0,0,127}));
  connect(hConWin.y, theConWin.Gc)
    annotation (Line(points={{20,33.6},{20,26}},         color={0,0,127}));
  connect(weaBus.TBlaSky, eqAirTemp.TBlaSky)
    annotation (Line(
    points={{-1,100},{0,100},{0,84},{-84,84},{-84,-4},{-38,-4}},
    color={255,204,51},
    thickness=0.5), Text(
    textString="%first",
    index=-1,
    extent={{-6,3},{-6,3}}));
  connect(macConv.port, thermalZoneFourElements.intGainsConv)
    annotation (
    Line(points={{20,-70},{84,-70},{84,20},{80,20}},          color={191,
    0,0}));
  connect(perCon.port, thermalZoneFourElements.intGainsConv)
    annotation (
    Line(points={{20,-50},{84,-50},{84,20},{80,20}}, color={191,0,0}));
  connect(preTemFloor.port, thermalZoneFourElements.floor)
    annotation (Line(points={{57,-6},{56,-6},{56,-2}}, color={191,0,0}));
  connect(TSoil.y, preTemFloor.T)
  annotation (Line(points={{44.4,-22},{57,-22},{57,-19.2}}, color={0,0,127}));
  connect(preTemRoof.port, theConRoof.fluid)
    annotation (Line(points={{55,58},{55,52}},         color={191,0,0}));
  connect(theConRoof.solid, thermalZoneFourElements.roof)
    annotation (Line(points={{55,42},{54.9,42},{54.9,34}}, color={191,0,0}));
  connect(eqAirTempVDI.TEqAir, preTemRoof.T)
    annotation (Line(
    points={{39,84},{55,84},{55,71.2}}, color={0,0,127}));
  connect(theConRoof.Gc, hConRoof.y)
    annotation (Line(points={{60,47},{69.6,47}},          color={0,0,127}));
  connect(eqAirTempVDI.TDryBul, eqAirTemp.TDryBul)
    annotation (Line(points={{16,78},{-90,78},{-90,-10},{-38,-10}},
    color={255,204,51},
      thickness=0.5));
  connect(eqAirTempVDI.TBlaSky, eqAirTemp.TBlaSky)
    annotation (Line(points={{16,84},{-84,84},{-84,-4},{-38,-4}},
    color={255,204,51},
      thickness=0.5));
  connect(eqAirTempVDI.HSol[1], weaBus.HGloHor)
    annotation (Line(points={{16,90},{4,90},{4,100},{-1,100}},
    color={255,204,51},
      thickness=0.5),Text(
    textString="%second",
    index=1,
    extent={{6,3},{6,3}}));
  connect(HDirTil.inc, corGDouPan.inc)
    annotation (Line(points={{-59,58},{-22,58},{-22,50},{-8,50}},
    color={0,0,127}));
  connect(const1.y, eqAirTempVDI.sunblind[1])
    annotation (Line(points={{49.7,93},{44,93},{44,98},{28,98},{28,96}},
                                      color={0,0,127}));
  connect(corGDouPan.solarRadWinTrans, thermalZoneFourElements.solRad)
    annotation (Line(points={{15,56},{28,56},{28,31},{31,31}}, color={0,0,127}));
  connect(thermalZoneFourElements.TAir, spl.x) annotation (Line(points={{81,32},
          {90,32},{90,60},{66,60},{66,80},{74,80}}, color={0,0,127}));
  connect(spl.y, y)
    annotation (Line(points={{97,80},{110,80}}, color={0,0,127}));
  connect(port_a, thermalZoneFourElements.ports[1]) annotation (Line(points={{-100,
          -80},{69.475,-80},{69.475,-1.95}}, color={0,127,255}));
  connect(port_b, thermalZoneFourElements.ports[2]) annotation (Line(points={{100,
          -80},{72.525,-80},{72.525,-1.95}}, color={0,127,255}));
  connect(weaBus, HDirTil[1].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,62},{-80,62}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, HDirTil[2].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,62},{-80,62}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, HDifTil[1].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,30},{-80,30}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, HDifTil[2].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,30},{-80,30}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  annotation ( Documentation(info="<html>
  <p>This example shows the application of
  <a href=\"Buildings.ThermalZones.ReducedOrder.RC.FourElements\">
  Buildings.ThermalZones.ReducedOrder.RC.FourElements</a>
  in combination with
  <a href=\"Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow\">
 Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow</a>
  and
  <a href=\"Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane\">
  Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane</a>.
  Solar radiation on tilted surface is calculated using models of
  Buildings. The thermal zone is a simple room defined in Guideline
  VDI 6007 Part 1 (VDI, 2012). All models, parameters and inputs
  except sunblinds, separate handling of heat transfer through
  windows, an extra wall element for ground floor (with additional
  area), an extra wall element for roof (with additional area) and
  solar radiation are similar to the ones defined for the
  guideline&apos;s test room. For solar radiation, the example
  relies on the standard weather file in Buildings.</p>
  <p>The idea of the example is to show a typical application of
  all sub-models and to use the example in unit tests. The results
  are reasonable, but not related to any real use case or measurement
  data.</p>
  <h4>References</h4>
  <p>VDI. German Association of Engineers Guideline VDI
  6007-1 March 2012. Calculation of transient thermal response of
  rooms and buildings - modelling of rooms.</p>
  </html>", revisions="<html>
  <ul>
  <li>
  July 11, 2019, by Katharina Brinkmann:<br/>
  Renamed <code>alphaWall</code> to <code>hConWall</code>,
  <code>alphaRoof</code> to <code>hConRoof</code>,
  <code>alphaWin</code> to <code>hConWin</code>
  </li>
  <li>
  April 27, 2016, by Michael Wetter:<br/>
  Removed call to <code>Modelica.Utilities.Files.loadResource</code>
  as this did not work for the regression tests.
  </li>
  <li>February 25, 2016, by Moritz Lauster:<br/>
  Implemented.
  </li>
  </ul>
  </html>"),
  experiment(Tolerance=1e-6, StopTime=3.1536e+007, Interval=3600),
  __Dymola_Commands(file=
  "modelica://Buildings/Resources/Scripts/Dymola/ThermalZones/ReducedOrder/Examples/SimpleRoomFourElements.mos"
        "Simulate and plot"),
    Icon(graphics={
        Rectangle(
          extent={{-60,22},{60,-98}},
          lineColor={150,150,150},
          fillPattern=FillPattern.Sphere,
          fillColor={255,255,255}),
        Rectangle(
          extent={{-38,-18},{-18,2}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Polygon(
          points={{0,52},{-60,22},{60,22},{0,52}},
          lineColor={95,95,95},
          smooth=Smooth.None,
          fillPattern=FillPattern.Solid,
          fillColor={95,95,95}),
        Rectangle(
          extent={{-100,-84},{-60,-74}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={255,0,0},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{20,-16},{40,4}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{20,-78},{40,-58}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-38,-78},{-18,-58}},
          lineColor={255,255,255},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{60,-84},{100,-74}},
          lineColor={0,0,255},
          pattern=LinePattern.None,
          fillColor={0,0,255},
          fillPattern=FillPattern.Solid)}));
end SimpleRoomFourElements;
