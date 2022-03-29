within BICEPS.ThermalZones;
model SimpleRoomFourElements
  "A thermal zone with four heat conduction elements"
  extends Buildings.BaseClasses.BaseIconLow;
  replaceable package Medium=Buildings.Media.Air
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Load side medium";
  parameter Boolean biomimeticControl = true
    "True if biomimetic control is enabled. False for standard control practice.";
  parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
  parameter Real k(min=Modelica.Constants.small)=10
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  parameter Real kIntLoa=0.5 "Scaling rate for interior loads";
  // Diagnostics
   parameter Boolean show_T = false
    "= true, if actual temperature at port is computed"
    annotation (
      Dialog(tab="Advanced", group="Diagnostics"),
      HideResult=true);
  Medium.ThermodynamicState sta_a=
      Medium.setState_phX(port_a.p,
                          noEvent(actualStream(port_a.h_outflow)),
                          noEvent(actualStream(port_a.Xi_outflow))) if
         show_T "Medium properties in port_a";

  Medium.ThermodynamicState sta_b=
      Medium.setState_phX(port_b.p,
                          noEvent(actualStream(port_b.h_outflow)),
                          noEvent(actualStream(port_b.Xi_outflow))) if
          show_T "Medium properties in port_b";
  Buildings.BoundaryConditions.SolarIrradiation.DiffusePerez HDifTil[4](
    each outSkyCon=true,
    each outGroCon=true,
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={0,1.5707963267949,-1.5707963267949,3.1415926535898})
    "Calculates diffuse solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
  Buildings.BoundaryConditions.SolarIrradiation.DirectTiltedSurface HDirTil[4](
    each til=1.5707963267949,
    each lat=0.87266462599716,
    azi={0,1.5707963267949,-1.5707963267949,3.1415926535898})
    "Calculates direct solar radiation on titled surface for both directions"
    annotation (Placement(transformation(extent={{-80,52},{-60,72}})));
  Buildings.ThermalZones.ReducedOrder.SolarGain.CorrectionGDoublePane corGDouPan(UWin=
        1.4002545917439533, n=4)
           "Correction factor for solar transmission"
    annotation (Placement(transformation(extent={{-6,46},{14,66}})));
  Buildings.ThermalZones.ReducedOrder.RC.FourElements thermalZoneFourElements(
    redeclare package Medium = Medium,
    VAir=366.3,
    hConExt=2.7,
    hConWin=2.7,
    gWin=0.6,
    ratioWinConRad=0.019999999999999997,
    nExt=1,
    RExt={0.00029535925449573306},
    CExt={33932211.77253235},
    hRad=5.0,
    AInt=451.77,
    hConInt=2.375675675675676,
    nInt=1,
    RInt={0.00014898377258981284},
    CInt={51799465.24399839},
    RWin=0.01929742189482449,
    RExtRem=0.015417680135418976,
    AFloor=79.56036000000002,
    hConFloor=1.7,
    nFloor=1,
    RFloor={0.0015452964217915787},
    RFloorRem=0.03785826582380714,
    CFloor={9285656.485114116},
    ARoof=85.61896200000001,
    hConRoof=1.6999999999999997,
    nRoof=1,
    RRoof={0.0007090222334459756},
    RRoofRem=0.039486473229807306,
    CRoof={1669857.3545976095},
    energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
    extWallRC(thermCapExt(each der_T(fixed=true))),
    intWallRC(thermCapInt(each der_T(fixed=true))),
    floorRC(thermCapExt(each der_T(fixed=true))),
    roofRC(thermCapExt(each der_T(fixed=true))),
    nOrientations=4,
    AWin={7.051275,7.051275,7.051275,7.051275},
    ATransparent={7.051275,7.051275,7.051275,7.051275},
    AExt={47.06955,47.06955,47.06955,47.06955},
    nPorts=2)
    "Thermal zone"
    annotation (Placement(transformation(extent={{32,-2},{80,34}})));
  Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007WithWindow
    eqAirTemp(
    wfGro=0,
    withLongwave=true,
    aExt=0.5,
    hConWallOut=20,
    hRad=5,
    hConWinOut=20,
    n=4,
    wfWall={0.25,0.25,0.25,0.25},
    wfWin={0.25,0.25,0.25,0.25},
    TGro=286.15) "Computes equivalent air temperature"
    annotation (Placement(transformation(extent={{-36,-14},{-16,6}})));
  Modelica.Blocks.Math.Add solRad[4]
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
    tableOnFile=true,
    table=[0,0,0,0; 3600,0,0,0; 7200,0,0,0; 10800,0,0,0; 14400,0,0,0; 18000,0,0,
        0; 21600,0,0,0; 25200,0,0,0; 25200,80,80,200; 28800,80,80,200; 32400,80,
        80,200; 36000,80,80,200; 39600,80,80,200; 43200,80,80,200; 46800,80,80,200;
        50400,80,80,200; 54000,80,80,200; 57600,80,80,200; 61200,80,80,200; 61200,
        0,0,0; 64800,0,0,0; 72000,0,0,0; 75600,0,0,0; 79200,0,0,0; 82800,0,0,0;
        86400,0,0,0],
    tableName="Internals",
    fileName=Modelica.Utilities.Files.loadResource(
        "modelica://BICEPS/Resources/Data/ThermalZones/InternalGains_DE.N.SFH.10.Gen.ReEx.001.001.txt"),
    columns={2,3,4},
    extrapolation=Modelica.Blocks.Types.Extrapolation.Periodic) "Table with profiles for persons (radiative and convective) and machines
    (convective)"
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Blocks.Sources.Constant const[4](each k=0)
    "Sets sunblind signal to zero (open)"
    annotation (Placement(transformation(extent={{-36,14},{-30,20}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(transformation(extent={{-18,84},{16,116}}),
        iconTransformation(extent={{-10,90},{10,110}})));
  Modelica.Thermal.HeatTransfer.Sources.PrescribedHeatFlow macConv
    "Convective heat flow of machines"
    annotation (Placement(transformation(extent={{0,-80},{20,-60}})));
  Modelica.Blocks.Sources.Constant hConWall(k=25.0*188.2782)
    "Outdoor coefficient of heat transfer for walls"
    annotation (Placement(transformation(extent={{-4,-4},{4,4}}, rotation=90,
    origin={20,-16})));
  Modelica.Blocks.Sources.Constant hConWin(k=24.999999999999996*28.2051)
    "Outdoor coefficient of heat transfer for windows"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},
    rotation=90,origin={20,38})));
  Buildings.HeatTransfer.Sources.PrescribedTemperature preTemFloor
    "Prescribed temperature for floor plate outdoor surface temperature"
    annotation (Placement(transformation(extent={{-6,-6},{6,6}},
    rotation=90,origin={57,-12})));
  Modelica.Blocks.Sources.Constant TSoil(k=286.15)
    "Outdoor surface temperature for floor plate"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},
    rotation=180,origin={40,-22})));
  Buildings.ThermalZones.ReducedOrder.EquivalentAirTemperature.VDI6007 eqAirTempVDI(
    aExt=0.5,
    n=2,
    wfWall={0.5,0.5},
    wfWin={0,0},
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
  Modelica.Blocks.Sources.Constant hConRoof(k=2140.47405)
    "Outdoor coefficient of heat transfer for roof"
    annotation (Placement(transformation(extent={{4,-4},{-4,4}},origin={74,47})));
  Modelica.Blocks.Sources.Constant const1[2](each k=0)
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
  Modelica.Blocks.Interfaces.RealOutput y if biomimeticControl "Control signal"
    annotation (Placement(transformation(extent={{100,70},{120,90}}),
        iconTransformation(
        extent={{100,60},{120,80}})));
  Utilities.Math.CubicHermite spl(
    final xMin=TMin,
    final xMax=TMax,
    final x0=T0,
    reverseActing=true) if biomimeticControl
    "Spline to calculate control signal"
    annotation (Placement(transformation(extent={{76,70},{96,90}})));

  Modelica.Blocks.Math.Gain gai[3](each k=kIntLoa) "Interior load scaling rate"
    annotation (Placement(transformation(extent={{-40,-60},{-20,-40}})));
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
    annotation (Line(points={{16,89},{4,89},{4,100},{-1,100}},
    color={255,204,51},
      thickness=0.5),Text(
    textString="%second",
    index=1,
    extent={{6,3},{6,3}}));
  connect(HDirTil.inc, corGDouPan.inc)
    annotation (Line(points={{-59,58},{-22,58},{-22,50},{-8,50}},
    color={0,0,127}));
  connect(corGDouPan.solarRadWinTrans, thermalZoneFourElements.solRad)
    annotation (Line(points={{15,56},{28,56},{28,31},{31,31}}, color={0,0,127}));
  connect(thermalZoneFourElements.TAir, spl.x) annotation (Line(points={{81,32},
          {84,32},{84,60},{66,60},{66,80},{74,80}}, color={0,0,127}));
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
  connect(const1.y, eqAirTempVDI.sunblind) annotation (Line(points={{49.7,93},{40,
          93},{40,100},{28,100},{28,96}}, color={0,0,127}));
  connect(weaBus, HDirTil[3].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,62},{-80,62}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, HDirTil[4].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,62},{-80,62}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, HDifTil[3].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,30},{-80,30}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus, HDifTil[4].weaBus) annotation (Line(
      points={{-1,100},{-8,100},{-8,94},{-96,94},{-96,30},{-80,30}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(weaBus.HGloHor, eqAirTempVDI.HSol[2]) annotation (Line(
      points={{-1,100},{-1,96},{4,96},{4,90},{16,90},{16,91}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-6,3},{-6,3}},
      horizontalAlignment=TextAlignment.Right));
  connect(intGai.y[1], gai[1].u)
    annotation (Line(points={{-59,-50},{-42,-50}}, color={0,0,127}));
  connect(intGai.y[2], gai[2].u)
    annotation (Line(points={{-59,-50},{-42,-50}}, color={0,0,127}));
  connect(intGai.y[3], gai[3].u)
    annotation (Line(points={{-59,-50},{-42,-50}}, color={0,0,127}));
  connect(gai[1].y, perRad.Q_flow) annotation (Line(points={{-19,-50},{-10,-50},
          {-10,-32},{0,-32}}, color={0,0,127}));
  connect(gai[2].y, perCon.Q_flow)
    annotation (Line(points={{-19,-50},{0,-50}}, color={0,0,127}));
  connect(gai[3].y, macConv.Q_flow) annotation (Line(points={{-19,-50},{-10,-50},
          {-10,-70},{0,-70}}, color={0,0,127}));
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
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={0,0,0},
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
          lineColor={0,0,0},
          pattern=LinePattern.None,
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid)}));
end SimpleRoomFourElements;
