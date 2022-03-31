within BICEPS.Electrical.Building.BaseClasses;
model ConnectedDevices
  "Model of distributed electrically connected devices 
  including producers, consumers, and storages."
  extends Buildings.BaseClasses.BaseIconLow;
  parameter Boolean biomimeticControl=true
    "True if biomimetic control is enabled. False for standard control practice.";
  parameter Integer nSto=1 "Number of storage connections";

  // Producer:
  parameter Modelica.SIunits.Angle lat "Latitude"
    annotation(Evaluate=true,Dialog(group="Orientation"));
  parameter Modelica.SIunits.Voltage V_nominal=208
    "Nominal voltage of the line";
  parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
  parameter Real k=100
    "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
  parameter Modelica.SIunits.Power PSun = 4000
    "Nominal power of the PV";
  parameter Modelica.SIunits.DensityOfHeatFlowRate W_m2_nominal = 1000
    "Nominal solar power per unit area";
  parameter Real eff_PV = 0.12*0.85*0.9
    "Nominal solar power conversion efficiency (this should consider converion efficiency, area covered, AC/DC losses)";
  parameter Modelica.SIunits.Area A_PV = PSun/eff_PV/W_m2_nominal
    "Nominal area of a P installation";
  // Storage: Battery
  parameter Real SOC_start=0.5 "Initial charge";
  parameter Modelica.SIunits.Power PBat = 5000
    "Nominal power charge/discharge rate of the battery";
  parameter Modelica.SIunits.Power PBatMax(min=0)=10000
    "Maximum power charge/discharge rate";
  parameter Modelica.SIunits.Power PBatMin(min=0)=100
    "Minimum power charge/discharge rate";
  // 50 kWh
  parameter Modelica.SIunits.Energy EBatMax = 180000000
    "Maximum energy capacity of the battery";
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terPro
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={60,110}), iconTransformation(extent={{30,100},{50,120}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terSto[nSto]
    annotation (Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=-90,
        origin={0,110}),  iconTransformation(extent={{-10,100},{10,120}})));
  Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terCon
    "Connector for consumers" annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=-90,
        origin={-40,110}), iconTransformation(extent={{-50,100},{-30,120}})));
  Modelica.Blocks.Interfaces.RealOutput yCon if biomimeticControl
    "Consumer control signal(s)"
    annotation (Placement(transformation(extent={{100,-10},{120,10}})));
  Modelica.Blocks.Interfaces.RealInput PNetIn "Net power input"
    annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.RealInput PCon "Total power of all consumers"
    annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
  Device.ConsumerThreePhaseBalanced con(
    each final biomimeticControl=biomimeticControl,
    each final V_nominal=V_nominal,
    each final tol=tol,
    each final k=k) "Consumers"
    annotation (Placement(transformation(extent={{-50,-50},{-30,-70}})));
  Modelica.Blocks.Interfaces.RealOutput ySto[nSto] if biomimeticControl
    "Storage control signal(s)"
    annotation (Placement(transformation(extent={{100,30},{120,50}})));
  Modelica.Blocks.Interfaces.RealOutput yPro if biomimeticControl
    "Producer control signal(s)"
    annotation (Placement(transformation(extent={{100,70},{120,90}})));
  Device.ProducerPV pv(
    each final biomimeticControl=biomimeticControl,
    each final V_nominal=V_nominal,
    each final tol=tol,
    each final k=k,
    each final PSun=PSun,
    each final lat=lat,
    each final W_m2_nominal=W_m2_nominal,
    each final eff_PV=eff_PV,
    each final A_PV=A_PV)
    annotation (Placement(transformation(extent={{50,60},{70,40}})));
  Device.StorageBattery bat[nSto](
    each final biomimeticControl=biomimeticControl,
    each final V_nominal=V_nominal,
    each final tol=tol,
    each final k=k,
    each final SOC_start=SOC_start,
    each final PBat=PBat,
    each final PMax=PBatMax,
    each final PMin=PBatMin,
    each final EBatMax=EBatMax)
    annotation (Placement(transformation(extent={{-10,-16},{10,4}})));
  Buildings.BoundaryConditions.WeatherData.Bus weaBus
    "Weather data bus"
    annotation (Placement(transformation(extent={{-100,80},{-60,120}}),
      iconTransformation(extent={{-100,90},{-80,110}})));

equation
  connect(PCon, con.P) annotation (Line(points={{-120,-60},{-52,-60},{-52,-60}},
        color={0,0,127}));
  connect(terCon, con.terminal)
    annotation (Line(points={{-40,110},{-40,-49.2}}, color={0,120,120}));
  connect(con.y, yCon) annotation (Line(points={{-29,-66},{94,-66},{94,0},{110,
          0}},
        color={0,0,127}));
  for i in 1:nSto loop
    connect(PNetIn, bat[i].PNetIn) annotation (Line(points={{-120,60},{-80,60},
            {-80,0},{-12,0}}, color={0,0,127}));
  end for;
  connect(weaBus, pv.weaBus) annotation (Line(
        points={{-80,100},{-80,70},{52,70},{52,60}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
  connect(bat.terminal, terSto)
    annotation (Line(points={{0,4.8},{0,110}},          color={0,120,120}));
  connect(bat.yOut, ySto) annotation (Line(points={{11,0},{90,0},{90,40},{110,
          40}},
        color={0,0,127}));
  connect(terPro, pv.terminal)
    annotation (Line(points={{60,110},{60,60.8}}, color={0,120,120}));
  connect(pv.yOut, yPro) annotation (Line(points={{71,44},{80,44},{80,80},{110,80}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
        Rectangle(
          extent={{-80,80},{80,-80}},
          lineColor={0,140,72},
          lineThickness=0.5),
        Polygon(points={{-56,40},{-24,40},{-12,16},{-24,-10},{-56,-10},{-68,16},
              {-56,40}},  lineColor={0,140,72}),
        Ellipse(extent={{26,-72},{-26,-22}},
                                          lineColor={0,140,72}),
        Polygon(
          points={{-20,-30},{0,0},{20,-30},{-20,-30}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{-20,-30},{0,0}}, color={0,140,72}),
        Line(points={{0,0},{20,-30}},  color={0,140,72}),
        Ellipse(
          extent={{78,6},{44,40}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{62,6},{34,40}},
          lineColor={0,140,72},
          fillColor={255,255,255},
          fillPattern=FillPattern.Solid,
          pattern=LinePattern.None),
        Line(points={{62,40},{14,40},{14,6},{62,6}},       color={0,140,72}),
        Line(points={{-40,40},{-40,100}}, color={0,0,0}),
        Line(points={{0,0},{0,100}},   color={0,0,0}),
        Line(points={{40,40},{40,100}}, color={0,0,0}),
        Polygon(
          points={{-40,-10},{-46,-20},{-34,-20},{-40,-10}},
          lineColor={0,0,0},
          fillColor={0,0,127},
          fillPattern=FillPattern.Solid),
        Line(points={{-100,-60},{-40,-60},{-40,-20}}, color={0,0,127}),
        Text(
          extent={{24,40},{58,6}},
          lineColor={0,140,72},
          textString="P"),
        Text(
          extent={{-16,-28},{18,-62}},
          lineColor={0,140,72},
          textString="S"),
        Text(
          extent={{-58,32},{-24,-2}},
          lineColor={0,140,72},
          textString="C")}),                              Diagram(
        coordinateSystem(preserveAspectRatio=false)));
end ConnectedDevices;
