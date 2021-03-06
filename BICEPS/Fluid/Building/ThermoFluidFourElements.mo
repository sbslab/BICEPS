within BICEPS.Fluid.Building;
model ThermoFluidFourElements "Thermofluid subsystem"
  extends Buildings.BaseClasses.BaseIconLow;
  replaceable package MediumWat=Buildings.Media.Water
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Medium in the building distribution system";
  replaceable package MediumAir=Buildings.Media.Air
    constrainedby Modelica.Media.Interfaces.PartialMedium
    "Load side medium";
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  // Diagnostic
  parameter Boolean show_T = false
    "= true, if actual temperature at port is computed"
    annotation (
      Dialog(tab="Advanced", group="Diagnostics"),
      HideResult=true);
  parameter Modelica.SIunits.HeatFlowRate QHea_flow_nominal=0
    "Nominal heating capacity (>=0)";
  parameter Real COP_nominal "Heat pump COP";
  parameter Modelica.SIunits.Pressure dp_nominal(displayUnit="Pa") = 50000
    "Pressure difference at nominal flow rate (for each flow leg)"
    annotation(Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.TemperatureDifference dT_nominal(min=0) = 5
    "Water temperature drop/increase accross load and source-side HX (always positive)"
    annotation (Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.Temperature TDisWatMin=279.15
    "District water minimum temperature"
    annotation (Dialog(group="DHC system"));
  parameter Modelica.SIunits.Temperature THeaWatSup_nominal=323.15
    "Heating water supply temperature"
    annotation (Dialog(group="Nominal condition"));
  final parameter Modelica.SIunits.Temperature THeaWatRet_nominal=
    THeaWatSup_nominal - dT_nominal
    "Heating water return temperature"
    annotation (Dialog(group="Nominal condition"));
  final parameter Modelica.SIunits.MassFlowRate mHeaWat_flow_nominal(min=0)=
    abs(QHea_flow_nominal / cpWat_default / (THeaWatSup_nominal - THeaWatRet_nominal))
    "Heating water mass flow rate"
    annotation(Dialog(group="Nominal condition"));
  parameter Modelica.SIunits.MassFlowRate mLoaHea_flow_nominal=1
    "Load side mass flow rate at nominal conditions in heating mode (single unit)"
    annotation (Dialog(group="Nominal condition"));
  constant Modelica.SIunits.SpecificHeatCapacity cpWat_default=
    MediumWat.specificHeatCapacityCp(MediumWat.setState_pTX(
      p = MediumWat.p_default,
      T = MediumWat.T_default))
    "Specific heat capacity of the fluid";
  parameter Boolean allowFlowReversal=false
    "Set to true to allow flow reversal on condenser side"
    annotation (Dialog(tab="Assumptions"), Evaluate=true);
  parameter Modelica.SIunits.Temperature TMin=288.15
    "Minimimum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature TMax=298.15
    "Maximum desired threshold for independent variable";
  parameter Modelica.SIunits.Temperature T0=293.15      "Nominal value for independent variable";
  parameter Real tSmo(
    final quantity="Time",
    final unit="s",
    min=1E-5)=30*60
    "Smoothing time for thermal-fluid control signal";
  Device.HeatPump heaPum(
    biomimeticControl=biomimeticControl,
    redeclare package Medium1 = MediumWat,
    redeclare package Medium2 = MediumWat,
    THeaWatSup_nominal=323.15,
    show_T1=show_T,
    show_T2=show_T,
    COP_nominal=COP_nominal,
    TCon_nominal=THeaWatSup_nominal,
    TEva_nominal=TDisWatMin - dT_nominal,
    Q1_flow_nominal=QHea_flow_nominal,
    dT1_nominal=dT_nominal,
    dT2_nominal=-dT_nominal,
    dp1_nominal=dp_nominal,
    dp2_nominal=dp_nominal)
    annotation (Placement(transformation(extent={{-10,-62},{-30,-42}})));
  Device.FanCoilWithDistributionPump fcu(
    biomimeticControl=biomimeticControl,
    redeclare package Medium1 = MediumWat,
    redeclare package Medium2 = MediumAir,
    show_T1=show_T,
    show_T2=show_T,
    m1_flow_nominal=mHeaWat_flow_nominal,
    m2_flow_nominal=mLoaHea_flow_nominal,
    dp1_nominal=100000,
    dp2_nominal=250,
    QHea_flow_nominal=QHea_flow_nominal,
    TMin=TMin,
    TMax=TMax,
    T0=T0) annotation (Placement(transformation(extent={{-30,-20},{-10,0}})));
  ThermalZones.SimpleRoomFourElements zon(
    biomimeticControl=biomimeticControl,
    TMin=TMin,
    TMax=TMax,
    T0=T0,
    k=100,
    show_T=show_T)
    annotation (Placement(transformation(extent={{-30,20},{-10,40}})));
  Modelica.Blocks.Interfaces.RealInput yEle if biomimeticControl
    "Relative exergetic potential of electrical subsystem"
    annotation (Placement(transformation(extent={{-140,50},{-100,90}})));
  Modelica.Blocks.Interfaces.RealOutput PHeaPum(
    final quantity="Power",
    final unit="W",
    min=0,
    displayUnit="kW") "Heat pump power"
    annotation (Placement(transformation(extent={{100,40},{120,60}})));
  Modelica.Fluid.Interfaces.FluidPort_a port_a(
    redeclare final package Medium = MediumWat,
    m_flow(min=if allowFlowReversal then -Modelica.Constants.inf else 0),
    h_outflow(start=MediumWat.h_default, nominal=MediumWat.h_default))
    "Fluid port inlet" annotation (Placement(transformation(extent={{-110,-70},{
            -90,-50}}), iconTransformation(extent={{-110,-70},{-90,-50}})));
  Modelica.Fluid.Interfaces.FluidPort_b port_b(
    redeclare final package Medium = MediumWat,
    m_flow(max=if allowFlowReversal then +Modelica.Constants.inf else 0),
    h_outflow(start=MediumWat.h_default, nominal=MediumWat.h_default))
    "Fluid port outlet" annotation (Placement(transformation(extent={{90,-70},{110,
            -50}}), iconTransformation(extent={{90,-70},{110,-50}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus "Weather data bus"
    annotation (Placement(transformation(extent={{-20,80},{20,120}}),
        iconTransformation(extent={{-10,90},{10,110}})));
  Controls.ThermoFluid conFlu(
    n=2,
    a={0.9,0.1},
    tSmo=tSmo) if                                biomimeticControl
    annotation (Placement(transformation(extent={{40,20},{60,40}})));

  Modelica.Blocks.Interfaces.RealOutput PPum(
    final quantity="Power",
    final unit="W",
    min=0,
    displayUnit="kW") "Pump power"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Modelica.Blocks.Sources.BooleanConstant heaPumOn(k=true)
    "Set heat pump to always on (for heating-season only simulation)"
    annotation (Placement(transformation(extent={{40,-90},{20,-70}})));
  Buildings.Controls.OBC.CDL.Continuous.Add addPumFan "Adder"
    annotation (Placement(transformation(extent={{60,70},{80,90}})));
equation
  connect(port_a, heaPum.port_a2)
    annotation (Line(points={{-100,-60},{-30,-60}}, color={0,127,255}));
  connect(heaPum.port_b2, port_b)
    annotation (Line(points={{-10,-60},{100,-60}}, color={0,127,255}));
  connect(heaPum.port_b1, fcu.port_a1) annotation (Line(points={{-30,-50},{-40,
          -50},{-40,-16},{-30,-16}},
                                color={0,127,255}));
  connect(fcu.port_b1, heaPum.port_a1) annotation (Line(points={{-10,-16},{0,
          -16},{0,-50},{-10,-50}},
                              color={0,127,255}));
  connect(fcu.port_b2, zon.port_a) annotation (Line(points={{-30,-6},{-40,-6},{
          -40,22},{-30,22}},
                         color={0,127,255}));
  connect(zon.port_b, fcu.port_a2) annotation (Line(points={{-10,22},{0,22},{0,
          -6},{-10,-6}},
                     color={0,127,255}));
  connect(weaBus, zon.weaBus) annotation (Line(
      points={{0,100},{0,92},{-20,92},{-20,40}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(heaPum.PHea, PHeaPum) annotation (Line(points={{-32,-42},{-60,-42},{
          -60,50},{110,50}},
                         color={0,0,127}));
  connect(yEle, conFlu.yIn[1]) annotation (Line(points={{-120,70},{34,70},{34,32},
          {40,32},{40,29}}, color={0,0,127}));
  connect(zon.y, conFlu.yIn[2]) annotation (Line(points={{-9,37},{20,37},{20,30},
          {40,30},{40,31}}, color={0,0,127}));
  connect(conFlu.yOut, heaPum.yHeaPum) annotation (Line(points={{61,30},{80,30},
          {80,-46},{-9,-46}}, color={0,0,127}));
  connect(conFlu.yOut, fcu.y) annotation (Line(points={{61,30},{80,30},{80,4},{
          -36,4},{-36,-2},{-32,-2}},   color={0,0,127}));
  connect(heaPumOn.y, heaPum.u) annotation (Line(points={{19,-80},{2,-80},{2,
          -42},{-9,-42}}, color={255,0,255}));
  connect(addPumFan.y, PPum)
    annotation (Line(points={{82,80},{110,80}}, color={0,0,127}));
  connect(heaPum.PPum, addPumFan.u1) annotation (Line(points={{-32,-46},{-64,
          -46},{-64,86},{58,86}}, color={0,0,127}));
  connect(fcu.PFan, addPumFan.u2) annotation (Line(points={{-8,-4},{10,-4},{10,
          74},{58,74}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(extent={{-40,22},{40,-58}}, lineColor={0,0,0}),
        Line(points={{-40,22},{0,62},{40,22}}, color={0,0,0})}),
      Diagram(coordinateSystem(preserveAspectRatio=false)));
end ThermoFluidFourElements;
