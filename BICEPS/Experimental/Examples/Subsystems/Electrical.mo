within BICEPS.Experimental.Examples.Subsystems;
model Electrical
  extends Buildings.BaseClasses.BaseIconLow;
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Real k=100
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  parameter Modelica.SIunits.Frequency f = 60 "Nominal grid frequency";
  parameter Modelica.SIunits.Power PLoa_nominal = 5000
    "Nominal power of a load";
  parameter Modelica.SIunits.Power PWin = PLoa_nominal*1
    "Nominal power of the wind turbine";
  parameter Modelica.SIunits.Power PSun = PLoa_nominal*1
    "Nominal power of the PV";
  parameter Modelica.SIunits.Angle lat "Latitude"
    annotation(Evaluate=true,Dialog(group="Orientation"));
  parameter Modelica.SIunits.DensityOfHeatFlowRate W_m2_nominal = 1000
    "Nominal solar power per unit area";
  parameter Real eff_PV = 0.12*0.85*0.9
    "Nominal solar power conversion efficiency (this should consider converion efficiency, area covered, AC/DC losses)";
  parameter Modelica.SIunits.Area A_PV = PSun/eff_PV/W_m2_nominal
    "Nominal area of a P installation";
  parameter Modelica.SIunits.Power PBat = PLoa_nominal/10
    "Nominal power charge/discharge rate of the battery";
  // 50 kWh
  parameter Modelica.SIunits.Energy EBatMax = 180000000
    "Maximum energy capacity of the battery";
  Modelica.Blocks.Interfaces.RealInput PHeaPum(
    final quantity="Power",
    final unit = "W",
    min=0,
    displayUnit = "kW") "Heat pump power"
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
  Modelica.Blocks.Interfaces.RealOutput yEle
    "Electrical subsystem control signal "
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linGri(
    l=1500,
    P_nominal=PWin + PSun + PBat + PLoa_nominal,
    V_nominal=V_nominal) "Grid power line"
    annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Storage.Battery bat(
    redeclare package PhaseSystem =
      Buildings.Electrical.PhaseSystems.ThreePhase_dq,
    SOC_start=0.5,
    EMax(displayUnit="J") = EBatMax,
    V_nominal=V_nominal,
    initMode=Buildings.Electrical.Types.InitMode.zero_current)
    annotation (Placement(transformation(extent={{-60,-40},{-80,-60}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Loads.Inductive loa(
    linearized=false,
    mode=Buildings.Electrical.Types.Load.VariableZ_P_input)
    annotation (Placement(transformation(extent={{-48,-10},{-68,10}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
    annotation (Placement(transformation(extent={{-120,60},{-100,80}}),
                              iconTransformation(extent={{-120,60},{-100,80}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus
    "Weather data bus"
    annotation (Placement(transformation(extent={{-20,80},{20,120}}),
      iconTransformation(extent={{-10,90},{10,110}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linWin(
    l=300,
    P_nominal=PWin,
    V_nominal=V_nominal)
    "Wind electrical line"
    annotation (Placement(transformation(extent={{30,60},{10,80}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linPV(
    l=300,
    P_nominal=PSun,
    V_nominal=V_nominal)
    "PV electrical line"
    annotation (Placement(transformation(extent={{30,20},{10,40}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linHP(
    l=10,
    P_nominal=PLoa_nominal,
    V_nominal=V_nominal)
    "Heat pump electrical line"
    annotation (Placement(transformation(extent={{-10,-10},{-30,10}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linBat(
    l=10,
    P_nominal=PBat,
    V_nominal=V_nominal)
    "Battery electrical line"
    annotation (Placement(transformation(extent={{-10,-60},{-30,-40}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.PVSimpleOriented pv(
    eta_DCAC=0.89,
    A=A_PV,
    fAct=0.9,
    eta=0.12,
    linearized=false,
    V_nominal=V_nominal,
    pf=0.85,
    lat=lat,
    azi=Buildings.Types.Azimuth.S,
    til=0.5235987755983) "PV"
    annotation (Placement(transformation(extent={{62,20},{82,40}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Sources.WindTurbine winTur(
    V_nominal=V_nominal,
    h=15,
    hRef=10,
    pf=0.94,
    eta_DCAC=0.92,
    nWin=0.4,
    tableOnFile=false,
    scale=PWin) "Wind turbine model"
    annotation (Placement(transformation(extent={{60,60},{80,80}})));
  Sensors.RelativeElectricalExergyPotential senLoa(
    tol=tol,
    v0=V_nominal,
    k=k) "Control signal load"
    annotation (Placement(transformation(extent={{-50,10},{-30,30}})));
  Sensors.RelativeElectricalExergyPotential senBat(
    tol=tol,
    v0=V_nominal,
    k=k) "Control signal battery"
    annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
  Sensors.RelativeElectricalExergyPotential senPV(
    tol=tol,
    v0=V_nominal,
    k=k) "Control signal pv"
    annotation (Placement(transformation(extent={{50,20},{30,0}})));
  Sensors.RelativeElectricalExergyPotential senWin(
    tol=tol,
    v0=V_nominal,
    k=k) "Control signal wind turbine"
    annotation (Placement(transformation(extent={{50,60},{30,40}})));
  Controls.SubsystemElectrical conSubEle(n=4)
    annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
  Controls.Battery conBat(EMax=EBatMax, P_nominal=PBat)
    "Battery controller"
    annotation (Placement(transformation(extent={{-40,-90},{-60,-70}})));
  Modelica.Blocks.Math.Gain inv(k=-1) "Invert to be negative (consumption)"
    annotation (Placement(transformation(extent={{-94,-10},{-74,10}})));
equation
  connect(terminal, linGri.terminal_n)
    annotation (Line(points={{-110,70},{-60,70}}, color={0,120,120}));
  connect(linGri.terminal_p, linWin.terminal_p)
    annotation (Line(points={{-40,70},{10,70}}, color={0,120,120}));
  connect(linPV.terminal_n, pv.terminal)
    annotation (Line(points={{30,30},{62,30}}, color={0,120,120}));
  connect(weaBus, pv.weaBus) annotation (Line(
      points={{0,100},{0,90},{90,90},{90,50},{72,50},{72,39}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(linWin.terminal_n, winTur.terminal)
    annotation (Line(points={{30,70},{60,70}}, color={0,120,120}));
  connect(weaBus.winSpe, winTur.vWin) annotation (Line(
      points={{0,100},{0,90},{70,90},{70,82}},
      color={255,204,51},
      thickness=0.5), Text(
      string="%first",
      index=-1,
      extent={{-3,6},{-3,6}},
      horizontalAlignment=TextAlignment.Right));
  connect(senLoa.terminal, loa.terminal)
    annotation (Line(points={{-40,10},{-40,0},{-48,0}}, color={0,120,120}));
  connect(senBat.terminal, bat.terminal) annotation (Line(points={{-40,-40},{-40,
          -50},{-60,-50}}, color={0,120,120}));
  connect(senPV.terminal, pv.terminal)
    annotation (Line(points={{40,20},{40,30},{62,30}}, color={0,120,120}));
  connect(senWin.terminal, winTur.terminal)
    annotation (Line(points={{40,60},{40,70},{60,70}}, color={0,120,120}));
  connect(conSubEle.yOut, yEle) annotation (Line(points={{41,-30},{90,-30},{90,0},
          {110,0}}, color={0,0,127}));
  connect(conSubEle.yOut, conBat.yNetPow) annotation (Line(points={{41,-30},{50,
          -30},{50,-74},{-38,-74}}, color={0,0,127}));
  connect(bat.SOC, conBat.soc) annotation (Line(points={{-81,-56},{-90,-56},{-90,
          -96},{-30,-96},{-30,-86},{-38,-86}}, color={0,0,127}));
  connect(conBat.P, bat.P)
    annotation (Line(points={{-61,-80},{-70,-80},{-70,-60}}, color={0,0,127}));
  connect(senWin.y, conSubEle.yIn[2]) annotation (Line(points={{29,42},{4,42},{
          4,-30.5},{20.2,-30.5}},
                                color={0,0,127}));
  connect(senPV.y, conSubEle.yIn[3]) annotation (Line(points={{29,2},{4,2},{4,
          -29.5},{20.2,-29.5}},
                       color={0,0,127}));
  connect(senBat.y, conSubEle.yIn[4]) annotation (Line(points={{-29,-22},{4,-22},
          {4,-28.5},{20.2,-28.5}}, color={0,0,127}));
  connect(senLoa.y, conSubEle.yIn[1]) annotation (Line(points={{-29,28},{4,28},
          {4,-31.5},{20.2,-31.5}},color={0,0,127}));
  connect(loa.terminal, linHP.terminal_p)
    annotation (Line(points={{-48,0},{-30,0}}, color={0,120,120}));
  connect(linPV.terminal_p, linGri.terminal_p) annotation (Line(points={{10,30},
          {0,30},{0,70},{-40,70}}, color={0,120,120}));
  connect(linHP.terminal_n, linGri.terminal_p) annotation (Line(points={{-10,0},
          {0,0},{0,70},{-40,70}}, color={0,120,120}));
  connect(linBat.terminal_p, bat.terminal)
    annotation (Line(points={{-30,-50},{-60,-50}}, color={0,120,120}));
  connect(linBat.terminal_n, linGri.terminal_p) annotation (Line(points={{-10,-50},
          {0,-50},{0,70},{-40,70}}, color={0,120,120}));
  connect(PHeaPum, inv.u)
    annotation (Line(points={{-120,0},{-96,0}}, color={0,0,127}));
  connect(inv.y, loa.Pow)
    annotation (Line(points={{-73,0},{-68,0}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false),
    graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,0,0},
          fillColor={0,140,72},
          fillPattern=FillPattern.Solid),
                             Polygon(
        points={{-50,-76},{-34,-26},{0,-58},{-50,-76}},
        lineColor={0,0,0},
        smooth=Smooth.None,
        fillPattern=FillPattern.Solid,
        fillColor={0,0,0}),      Line(
        points={{40,78},{-28,10},{32,10},{-50,-76},{-50,-76}},
        color={0,0,0},
        smooth=Smooth.None)}));
end Electrical;
