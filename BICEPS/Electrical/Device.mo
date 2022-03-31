within BICEPS.Electrical;
package Device "Package with electrical devices"
  model ConsumerThreePhaseBalanced
    "Generic model for an electrical consumer with a three phase balanced load"
    extends Buildings.BaseClasses.BaseIcon;
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Modelica.SIunits.Voltage V_nominal=208
      "Nominal voltage of the line";
    parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
    parameter Real k=100
      "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.Inductive loa(linearized=false,
        mode=Buildings.Electrical.Types.Load.VariableZ_P_input)
      annotation (Placement(transformation(extent={{-20,-10},{-40,10}})));
    Experimental.Examples.Sensors.RelativeElectricalExergyPotential senV(
      tol=tol,
      v0=V_nominal,
      k=k) if biomimeticControl
      "Control signal load"
      annotation (Placement(transformation(extent={{-10,42},{10,62}})));
    Modelica.Blocks.Math.Gain inv(k=-1) "Invert to be negative (consumption)"
      annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    Modelica.Blocks.Interfaces.RealInput P(
      final quantity="Power",
      final unit="W",
      min=0,
      displayUnit="kW") "Consumer power (positive value)"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terminal
      "Generalized electric terminal"
      annotation (Placement(transformation(extent={{-4,-112},{12,-96}}),
          iconTransformation(extent={{-8,-116},{8,-100}})));
    Modelica.Blocks.Interfaces.RealOutput y if biomimeticControl
      "Control signal"
      annotation (Placement(transformation(extent={{100,50},{120,70}})));
  equation
    connect(P, inv.u)
      annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
    connect(inv.y,loa. Pow)
      annotation (Line(points={{-59,0},{-40,0}}, color={0,0,127}));
    connect(loa.terminal, terminal) annotation (Line(points={{-20,0},{0,0},{0,-104},
            {4,-104}}, color={0,120,120}));
    connect(senV.terminal, terminal)
      annotation (Line(points={{0,42},{0,-104},{4,-104}}, color={0,120,120}));
    connect(senV.y, y) annotation (Line(points={{11,60},{110,60}},
          color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Polygon(
            points={{-54,50},{30,-20},{-54,50}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-10,20},{10,-20}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            origin={-30,-40},
            rotation=180),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={-30,0},
            rotation=360),
          Rectangle(
            extent={{-10,20},{10,-20}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            origin={0,-40},
            rotation=180),
          Rectangle(
            extent={{-10,20},{10,-20}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            origin={30,-40},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-4.89859e-15,40}},
                                           color={0,0,0},
            origin={0,-60},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-1.46957e-15,12}},
                                           color={0,0,0},
            origin={-30,-60},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-1.46957e-15,12}},
                                           color={0,0,0},
            origin={30,-60},
            rotation=180),
          Polygon(
            points={{-40,68},{40,68},{80,0},{40,-72},{-40,-72},{-80,0},{-40,68}},
            lineColor={0,140,72},
            lineThickness=0.5),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={-30,20},
            rotation=360),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={-30,40},
            rotation=360),
          Rectangle(
            extent={{-30,52},{-18,-10}},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
            Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                           color={0,0,0},
            origin={-30,-10},
            rotation=180),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={0,40},
            rotation=360),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={0,20},
            rotation=360),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={0,0},
            rotation=360),
          Rectangle(
            extent={{0,52},{12,-10}},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={30,40},
            rotation=360),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={30,20},
            rotation=360),
          Ellipse(extent={{-10,-10},{10,10}},
            origin={30,0},
            rotation=360),
          Rectangle(
            extent={{30,52},{42,-10}},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
            Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                           color={0,0,0},
            origin={0,-10},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                           color={0,0,0},
            origin={30,-10},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-2.20436e-15,18}},
                                           color={0,0,0},
            origin={0,68},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                           color={0,0,0},
            origin={30,60},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-1.22464e-15,10}},
                                           color={0,0,0},
            origin={-30,60},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{30,8}},
                                           color={0,0,0},
            origin={0,68},
            rotation=180),
            Line(points={{-6.85214e-44,-8.39117e-60},{-30,8}},
                                           color={0,0,0},
            origin={0,68},
            rotation=180),
          Line(points={{-100,0},{-80,0}}, color={0,0,127}),
          Line(points={{100,60},{98,60},{80,60},{56,42}}, color={0,0,127})}),
                             Diagram(coordinateSystem(preserveAspectRatio=false)));
  end ConsumerThreePhaseBalanced;

  model Panel "Generic model for an electrical panel"
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Integer nSto=1 "Number of storage connections";
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terGri
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={0,110}), iconTransformation(extent={{-10,100},{10,120}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terPro
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={-60,-110}), iconTransformation(extent={{-50,-120},{-30,-100}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terSto[nSto]
      annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={0,-110}), iconTransformation(extent={{-10,-120},{10,-100}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_n terCon
      "Connector for consumers" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={60,-110}), iconTransformation(extent={{30,-120},{50,-100}})));
    Modelica.Blocks.Interfaces.RealInput yCon if biomimeticControl
      "Consumer control signal(s)"
      annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
    Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
      "Output control signal"
      annotation (Placement(transformation(extent={{100,50},{120,70}})));
    Buildings.Electrical.AC.OnePhase.Sensors.GeneralizedSensor met "Main meter"
      annotation (Placement(transformation(
          extent={{-10,10},{10,-10}},
          rotation=-90,
          origin={0,40})));
    Controls.Panel con(n=nSto + 2) if
                                  biomimeticControl
      "Controller"
      annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
    Modelica.Blocks.Interfaces.RealInput ySto[nSto] if biomimeticControl
      "Storage control signal(s)"
      annotation (Placement(transformation(extent={{-140,20},{-100,60}})));
    Modelica.Blocks.Interfaces.RealInput yPro if biomimeticControl
      "Producer control signal(s)"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Interfaces.RealOutput PNetOut "Net power output"
      annotation (Placement(transformation(extent={{100,10},{120,30}})));
    BICEPS.Electrical.Sensors.TotalPower senPro "Sensor on producers" annotation (
       Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-60,-50})));
    BICEPS.Electrical.Sensors.TotalPower senCon "Sensor on consumers" annotation (
       Placement(transformation(
          extent={{10,10},{-10,-10}},
          rotation=90,
          origin={60,-50})));
    Modelica.Blocks.Math.Add PNet(k1=+1, k2=-1)
      "Net power supply/demand only"
      annotation (Placement(transformation(extent={{40,10},{60,30}})));
  equation
    connect(terGri, met.terminal_n)
      annotation (Line(points={{0,110},{0,50},{1.77636e-15,50}},
                                                color={0,120,120}));
    connect(senPro.terminal_p, met.terminal_p)
      annotation (Line(points={{-60,-40},{-60,-30},{0,-30},{0,30}},
        color={0,120,120}));
    connect(yPro, con.yIn[1])
      annotation (Line(
       points={{-120,0},{-70,0},{-70,60},{-59.8,60}},
       color={0,0,127}));
    for i in 1:nSto loop
      connect(met.terminal_p, terSto[i])
        annotation (Line(points={{-1.77636e-15,30},{-1.77636e-15,-110},{0,-110}},
                                                            color={0,120,120}));
      connect(ySto[i], con.yIn[i+2])
        annotation (Line(points={{-120,40},{-70,40},{-70,60},{-59.8,60}},
           color={0,0,127}));
    end for;
    connect(senCon.terminal_n, met.terminal_p)
      annotation (Line(points={{60,-40},{60,-30},{0,-30},{0,30},{-1.77636e-15,
            30}},color={0,120,120}));
    connect(yCon, con.yIn[2])
      annotation (Line(points={{-120,80},{-70,80},{-70,60},{-59.8,60}}, color={0,0,127}));
    connect(con.yOut, yOut)
      annotation (Line(points={{-39,60},{110,60}}, color={0,0,127}));
    connect(terPro, senPro.terminal_n)
      annotation (Line(points={{-60,-110},{-60,-60}}, color={0,120,120}));
    connect(terCon, senCon.terminal_p)
      annotation (Line(points={{60,-110},{60,-60}}, color={0,120,120}));
    connect(PNet.y, PNetOut)
      annotation (Line(points={{61,20},{110,20}}, color={0,0,127}));
    connect(senPro.P, PNet.u1) annotation (Line(points={{-51,-50},{-20,-50},{-20,26},
            {38,26}}, color={0,0,127}));
    connect(senCon.P, PNet.u2) annotation (Line(points={{51,-50},{20,-50},{20,14},
            {38,14}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Line(points={{0,-24},{0,-32}}, color={0,0,0}),
            Rectangle(
            extent={{-60,80},{60,-80}},
            lineColor={0,140,72},
            lineThickness=0.5),
          Rectangle(
            extent={{-26,24},{24,-24}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Text(
            extent={{-22,20},{20,-20}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            textString="M"),
          Ellipse(extent={{-22,-20},{20,20}}, lineColor={0,0,0}),
          Line(points={{0,-36},{16,-64}}, color={0,0,0}),
          Ellipse(
            extent={{-4,-32},{4,-40}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{-4,-60},{4,-68}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid),
          Line(points={{0,24},{0,100}}, color={0,0,0}),
          Rectangle(
            extent={{-52,70},{50,30}},
            lineColor={0,0,0},
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),     Text(
            extent={{-56,80},{56,28}},
            lineColor={0,0,255},
            textString="%name"),
          Line(points={{0,-68},{0,-100}}, color={0,0,0}),
          Line(points={{40,-100},{0,-80},{-40,-100}}, color={0,0,0})}),
                                   Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Panel;

  model ProducerPV "PV subsystem"
    extends Buildings.BaseClasses.BaseIcon;
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Modelica.SIunits.Voltage V_nominal=208
      "Nominal voltage of the line";
    parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
    parameter Real k=100
      "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    parameter Modelica.SIunits.Power PSun = 4000
      "Nominal power of the PV";
    parameter Modelica.SIunits.Angle lat "Latitude"
      annotation(Evaluate=true,Dialog(group="Orientation"));
    parameter Modelica.SIunits.DensityOfHeatFlowRate W_m2_nominal = 1000
      "Nominal solar power per unit area";
    parameter Real eff_PV = 0.12*0.85*0.9
      "Nominal solar power conversion efficiency (this should consider converion efficiency, area covered, AC/DC losses)";
    parameter Modelica.SIunits.Area A_PV = PSun/eff_PV/W_m2_nominal
      "Nominal area of a P installation";
    Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
      "Output control signal"
      annotation (Placement(transformation(extent={{100,50},{120,70}})));
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
      annotation (Placement(transformation(extent={{-20,-30},{-40,-10}})));
    Experimental.Examples.Sensors.RelativeElectricalExergyPotential senPV(
      tol=tol,
      v0=V_nominal,
      k=k) if biomimeticControl
      "Control signal pv"
      annotation (Placement(transformation(extent={{-10,42},{10,62}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
      "Generalized electric terminal"
      annotation (Placement(transformation(extent={{-8,-116},{8,-100}}),
          iconTransformation(extent={{-8,-116},{8,-100}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      "Weather data bus"
      annotation (Placement(transformation(extent={{-100,-80},{-60,-120}}),
        iconTransformation(extent={{-90,-90},{-70,-110}})));
  equation
    connect(senPV.terminal, pv.terminal)
      annotation (Line(points={{0,42},{0,-20},{-20,-20}},    color={0,120,120}));
    connect(pv.terminal, terminal)
      annotation (Line(points={{-20,-20},{0,-20},{0,-108},{0,-108}},
                                                           color={0,120,120}));
    connect(senPV.y, yOut) annotation (Line(points={{11,60},{110,60}},
                  color={0,0,127}));
    connect(weaBus, pv.weaBus) annotation (Line(
        points={{-80,-100},{-80,10},{-30,10},{-30,-11}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-6,40},{86,-60}},
            lineColor={0,140,72},
            lineThickness=0.5),
          Rectangle(
            extent={{-10,40},{38,-60}},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Polygon(
            points={{-72,-10},{-12,-10},{8,20},{-52,20},{-72,-10}},
            lineColor={0,0,0},
            fillColor={205,203,203},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-8,-10},{52,-10},{72,20},{12,20},{-8,-10}},
            lineColor={0,0,0},
            fillColor={205,203,203},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-30,-44},{30,-44},{50,-14},{-10,-14},{-30,-44}},
            lineColor={0,0,0},
            fillColor={205,203,203},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-80,-44},{-36,-44},{-16,-14},{-76,-14},{-79.3789,-19.0684},{-79.9957,
                -19.9936},{-80,-44}},
            lineColor={0,0,0},
            fillColor={205,203,203},
            fillPattern=FillPattern.Solid),
          Line(
            points={{40,40},{8,40},{-80,40},{-80,-60},{40,-60}},
            color={0,140,72},
            thickness=0.5),
          Line(points={{0,-60},{0,-76},{0,-100}}, color={0,0,0}),
          Polygon(
            points={{34,6},{58,6},{66,18},{42,18},{34,6}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{6,6},{30,6},{38,18},{14,18},{6,6}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{24,-8},{48,-8},{56,4},{32,4},{24,-8}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-4,-8},{20,-8},{28,4},{4,4},{-4,-8}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-26,-42},{-2,-42},{6,-30},{-18,-30},{-26,-42}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-16,-28},{8,-28},{16,-16},{-8,-16},{-16,-28}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{12,-28},{36,-28},{44,-16},{20,-16},{12,-28}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{2,-42},{26,-42},{34,-30},{10,-30},{2,-42}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-66,-8},{-42,-8},{-34,4},{-58,4},{-66,-8}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-56,6},{-32,6},{-24,18},{-48,18},{-56,6}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-30,6},{-6,6},{2,18},{-22,18},{-30,6}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-40,-8},{-16,-8},{-8,4},{-32,4},{-40,-8}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-80,-42},{-68,-42},{-60,-30},{-80,-30},{-80,-42}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-80,-28},{-58,-28},{-50,-16},{-74,-16},{-79.9844,-24.9766},{
                -80,-28}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-54,-28},{-30,-28},{-22,-16},{-46,-16},{-54,-28}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid),
          Polygon(
            points={{-64,-42},{-40,-42},{-32,-30},{-56,-30},{-64,-42}},
            lineColor={0,0,0},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid)}),
                              Diagram(coordinateSystem(preserveAspectRatio=false)));
  end ProducerPV;

  model ProducerWind "Wind subsystem"
    extends Buildings.BaseClasses.BaseIcon;
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Modelica.SIunits.Voltage V_nominal=208
      "Nominal voltage of the line";
    parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
    parameter Real k=100
      "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    parameter Modelica.SIunits.Power PWin
      "Nominal power of the wind turbine";
    parameter Modelica.SIunits.Angle lat "Latitude"
      annotation(Evaluate=true,Dialog(group="Orientation"));
    Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
      "Output control signal"
      annotation (Placement(transformation(extent={{100,50},{120,70}})));
    Experimental.Examples.Sensors.RelativeElectricalExergyPotential senWin(
      tol=tol,
      v0=V_nominal,
      k=k) if biomimeticControl
      "Control signal wind"
      annotation (Placement(transformation(extent={{-10,42},{10,62}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
      "Generalized electric terminal"
      annotation (Placement(transformation(extent={{-8,-116},{8,-100}}),
          iconTransformation(extent={{-8,-116},{8,-100}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      "Weather data bus"
      annotation (Placement(transformation(extent={{-100,-80},{-60,-120}}),
        iconTransformation(extent={{-90,-90},{-70,-110}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Sources.WindTurbine winTur(
      V_nominal=V_nominal,
      h=15,
      hRef=10,
      pf=0.94,
      eta_DCAC=0.92,
      nWin=0.4,
      tableOnFile=false,
      scale=PWin) "Wind turbine model"
      annotation (Placement(transformation(extent={{-20,-20},{-40,0}})));
  equation
    connect(senWin.y, yOut)
      annotation (Line(points={{11,60},{110,60}}, color={0,0,127}));
    connect(weaBus.winSpe, winTur.vWin) annotation (Line(
        points={{-80,-100},{-80,12},{-30,12},{-30,2}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(senWin.terminal, terminal)
      annotation (Line(points={{0,42},{0,-108},{0,-108}}, color={0,120,120}));
    connect(winTur.terminal, terminal) annotation (Line(points={{-20,-10},{0,-10},
            {0,-108},{0,-108}}, color={0,120,120}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-6,40},{86,-60}},
            lineColor={0,140,72},
            lineThickness=0.5),
          Rectangle(
            extent={{-10,40},{38,-60}},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Line(
            points={{40,40},{8,40},{-80,40},{-80,-60},{40,-60}},
            color={0,140,72},
            thickness=0.5),
          Line(points={{0,-60},{0,-76},{0,-100}}, color={0,0,0}),
          Polygon(
            points={{-40,-4},{-79.9844,19.3242},{-80,20},{-80.0273,20.627},{-36,2},
                {-40,-4}},
            smooth=Smooth.None,
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0}),
          Polygon(
            points={{-36,-4},{10,30},{-40,2},{-36,-4}},
            smooth=Smooth.None,
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0}),
          Polygon(
            points={{-42,-4},{-24,-56},{-36,0},{-42,-4}},
            smooth=Smooth.None,
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0}),
          Rectangle(
            extent={{-40,-2},{-36,-60}},
            fillColor={233,233,233},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0}),
          Polygon(
            points={{24,-14},{-20,22},{26,-8},{24,-14}},
            smooth=Smooth.None,
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid,
            origin={20,-2},
            rotation=90,
            lineColor={0,0,0}),
          Polygon(
            points={{-21,-17},{-7.03125,-7.10547},{-6.97266,-1.29297},{-25,-11},{
                -21,-17}},
            smooth=Smooth.None,
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid,
            origin={17,47},
            rotation=90,
            lineColor={0,0,0}),
          Polygon(
            points={{28,22},{77.6289,18.6914},{76.8984,19.9141},{30,28},{28,22}},
            smooth=Smooth.None,
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0}),
          Rectangle(
            extent={{30,24},{34,-60}},
            fillColor={233,233,233},
            fillPattern=FillPattern.Solid,
            lineColor={0,0,0}),
          Ellipse(
            extent={{-44,4},{-32,-8}},
            lineColor={0,0,0},
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid),
          Ellipse(
            extent={{26,30},{38,18}},
            lineColor={0,0,0},
            fillColor={222,222,222},
            fillPattern=FillPattern.Solid)}),
                              Diagram(coordinateSystem(preserveAspectRatio=false)));
  end ProducerWind;

  model StorageBattery
    "Model for a chemical battery for the electrical system"
    extends Buildings.BaseClasses.BaseIconLow;
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Modelica.SIunits.Voltage V_nominal=208
      "Nominal voltage of the line";
    parameter Real tol=0.05 "Tolerance allowed on nominal voltage control (5-10% typical)";
    parameter Real k=100
      "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    replaceable package PhaseSystem =
        Buildings.Electrical.PhaseSystems.OnePhase
      annotation (__Dymola_choicesAllMatching=true);
    parameter Real SOC_start=0.5 "Initial charge";
    parameter Modelica.SIunits.Power PBat = 5000
      "Nominal power charge/discharge rate of the battery";
    parameter Modelica.SIunits.Power PMax(min=0)=10000
      "Maximum power charge/discharge rate";
    parameter Modelica.SIunits.Power PMin(min=0)=100
      "Minimum power charge/discharge rate";
    // 50 kWh
    parameter Modelica.SIunits.Energy EBatMax = 180000000
      "Maximum energy capacity of the battery";
    Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
      "Output control signal"
      annotation (Placement(transformation(extent={{100,50},{120,70}})));
    Experimental.Examples.Sensors.RelativeElectricalExergyPotential senBat(
      tol=tol,
      v0=V_nominal,
      k=k) if biomimeticControl
      "Control signal battery"
      annotation (Placement(transformation(extent={{-10,42},{10,62}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Storage.Battery bat(
      redeclare package PhaseSystem = PhaseSystem,
      SOC_start=SOC_start,
      EMax(displayUnit="J") = EBatMax,
      V_nominal=V_nominal,
      initMode=Buildings.Electrical.Types.InitMode.zero_current)
      annotation (Placement(transformation(extent={{10,-20},{30,0}})));
    replaceable Controls.Battery2 con(
      EMax=EBatMax,
      P_nominal=PBat,
      PMax=PMax,
      PMin=PMin,
      riseTime=15)
      "Battery controller"
      annotation (Placement(transformation(extent={{-60,10},{-40,30}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
      "Generalized electric terminal"
      annotation (Placement(transformation(extent={{-12,-112},{4,-96}}),
          iconTransformation(extent={{-8,100},{8,116}})));
    Modelica.Blocks.Interfaces.RealInput PNetIn "Net power input"
      annotation (Placement(transformation(extent={{-140,40},{-100,80}})));

  equation
    connect(PNetIn, con.PNetIn) annotation (Line(points={{-120,60},{-70,60},{-70,
            26},{-62,26}}, color={0,0,127}));
    connect(senBat.terminal, terminal)
      annotation (Line(points={{0,42},{0,-104},{-4,-104}}, color={0,120,120}));
    connect(bat.terminal, terminal) annotation (Line(points={{10,-10},{0,-10},{0,-104},
            {-4,-104}}, color={0,120,120}));
    connect(con.P, bat.P)
      annotation (Line(points={{-39,20},{20,20},{20,0}}, color={0,0,127}));
    connect(bat.SOC, con.soc) annotation (Line(points={{31,-4},{40,-4},{40,-20},{-70,
            -20},{-70,16},{-62,16}}, color={0,0,127}));
    connect(senBat.y, yOut)
      annotation (Line(points={{11,60},{110,60}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Ellipse(
            extent={{-54,-74},{56,38}},
            lineColor={0,140,72},
            lineThickness=0.5),
          Line(
            points={{-70,0}},
            color={0,140,72},
            thickness=0.5),
          Polygon(
            points={{-40,20},{0,74},{40,22},{-40,20}},
            lineColor={0,140,72},
            lineThickness=0.5,
            fillColor={255,255,255},
            fillPattern=FillPattern.Solid,
            pattern=LinePattern.None),
          Line(
            points={{-40,20},{0,74}},
            color={0,140,72},
            thickness=0.5),
          Line(
            points={{40,22},{0,74}},
            color={0,140,72},
            thickness=0.5),
          Rectangle(
            extent={{40,-46},{-30,4}},
            lineColor={215,215,215},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-30,-30},{-40,-10}},
            lineColor={215,215,215},
            fillColor={215,215,215},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{20,-2},{36,-40}},
            lineColor={6,13,150},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid,
            radius=1),
          Rectangle(
            extent={{-26,-2},{-10,-40}},
            lineColor={6,13,150},
            radius=1),
          Line(points={{100,60},{70,60},{34,30}}, color={0,0,127}),
          Line(points={{0,4},{0,100}},            color={0,0,0}),
          Line(points={{-100,60},{-84,60},{-70,60},{-32,30}}, color={0,0,127}),
          Rectangle(
            extent={{-4,-2},{12,-40}},
            lineColor={6,13,150},
            fillColor={6,13,150},
            fillPattern=FillPattern.Solid,
            radius=1)}),      Diagram(coordinateSystem(preserveAspectRatio=false)));
  end StorageBattery;

  package Controls
    model Battery "Control for the battery energy storage system"
      extends Modelica.Blocks.Icons.Block;
      parameter Modelica.SIunits.Energy EMax(min=0, displayUnit="kWh")
        "Maximum available charge";
      parameter Modelica.SIunits.Power P_nominal(min=0)
        "Nominal power charge/discharge rate";
      parameter Modelica.SIunits.Power PMax(min=0)=10000
        "Maximum power charge/discharge rate";
      parameter Modelica.SIunits.Power PMin(min=0)=100
        "Minimum power charge/discharge rate";
      parameter Modelica.SIunits.Time riseTime=60
        "Rise time of the filter (time to reach 99.6 % of the transition speed)"
        annotation(Dialog(tab="Dynamics", group="Filtered transition speed"));
      Modelica.Blocks.Interfaces.RealInput PNetIn "Net power input"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Modelica.Blocks.Interfaces.RealInput soc "State of charge"
        annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
      Modelica.Blocks.Interfaces.RealOutput P(
        start = 0,
        final quantity="Power",
        final unit="W",
        displayUnit="KW") "Battery power (negative for discharge)"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Sources.Constant off(k=0)
        "Battery state of charge is at its limit and cannot charge/discharge further"
        annotation (Placement(transformation(extent={{30,-60},{50,-40}})));
      Buildings.Controls.OBC.CDL.Continuous.LessThreshold belCap(t=0.95, h=0.04)
        "Below SOC capacity. Hysteresis set for 10s cycles"
        annotation (Placement(transformation(extent={{-80,-50},{-60,-30}})));
      Buildings.Controls.OBC.CDL.Logical.Switch swi
        annotation (Placement(transformation(extent={{70,20},{90,40}})));
      Buildings.Controls.OBC.CDL.Logical.And cha "Charge"
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Buildings.Controls.OBC.CDL.Logical.And dis "Discharge"
        annotation (Placement(transformation(extent={{0,-20},{20,0}})));
      Buildings.Controls.OBC.CDL.Logical.Or chaOrDis "Charge or discharge"
        annotation (Placement(transformation(extent={{30,20},{50,40}})));
      Modelica.Blocks.Continuous.Filter fil(
        analogFilter=Modelica.Blocks.Types.AnalogFilter.CriticalDamping,
        filterType=Modelica.Blocks.Types.FilterType.LowPass,
        order=2,
        f_cut=5/(2*Modelica.Constants.pi*riseTime),
        init=Modelica.Blocks.Types.Init.InitialOutput)
        "Second order filter to approximate battery transitions between charge/off/discharge/off/charge"
        annotation (Placement(transformation(extent={{70,-10},{90,10}})));
      Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold notEmp(t=0.05, h=0.04)
        "Not empty."
        annotation (Placement(transformation(extent={{-80,-80},{-60,-60}})));
      Modelica.Blocks.Math.RealToBoolean grePowNomPos(threshold=P_nominal*1.05)
        "Greater than nominal power (positive)"
        annotation (Placement(transformation(extent={{-80,20},{-60,40}})));
      Modelica.Blocks.Math.RealToBoolean grePowNomNeg(threshold=-P_nominal*1.05)
        "Greater than nominal power (negative)"
        annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
      Buildings.Controls.OBC.CDL.Logical.Not lesPowNomNeg
        "Less than nominal power (negative)"
        annotation (Placement(transformation(extent={{-40,-20},{-20,0}})));
    equation
      connect(soc, belCap.u)
        annotation (Line(points={{-120,-40},{-82,-40}}, color={0,0,127}));
      connect(cha.u2, belCap.y) annotation (Line(points={{-2,22},{-50,22},{-50,-40},
              {-58,-40}},color={255,0,255}));
      connect(cha.y, chaOrDis.u1)
        annotation (Line(points={{22,30},{28,30}}, color={255,0,255}));
      connect(dis.y, chaOrDis.u2) annotation (Line(points={{22,-10},{24,-10},{24,22},
              {28,22}},                 color={255,0,255}));
      connect(chaOrDis.y, swi.u2)
        annotation (Line(points={{52,30},{68,30}}, color={255,0,255}));
      connect(off.y, swi.u3) annotation (Line(points={{51,-50},{60,-50},{60,22},{68,
              22}}, color={0,0,127}));
      connect(swi.y, fil.u) annotation (Line(points={{92,30},{94,30},{94,16},{64,16},
              {64,0},{68,0}}, color={0,0,127}));
      connect(fil.y, P) annotation (Line(points={{91,0},{110,0}}, color={0,0,127}));
      connect(soc, notEmp.u) annotation (Line(points={{-120,-40},{-90,-40},{-90,-70},
              {-82,-70}}, color={0,0,127}));
      connect(notEmp.y, dis.u2) annotation (Line(points={{-58,-70},{-10,-70},{-10,-18},
              {-2,-18}}, color={255,0,255}));
      connect(PNetIn, grePowNomPos.u) annotation (Line(points={{-120,60},{-90,60},{-90,
              30},{-82,30}}, color={0,0,127}));
      connect(grePowNomNeg.y, lesPowNomNeg.u)
        annotation (Line(points={{-59,-10},{-42,-10}}, color={255,0,255}));
      connect(grePowNomPos.y, cha.u1)
        annotation (Line(points={{-59,30},{-2,30}}, color={255,0,255}));
      connect(PNetIn, grePowNomNeg.u) annotation (Line(points={{-120,60},{-90,60},{-90,
              -10},{-82,-10}}, color={0,0,127}));
      connect(dis.u1, lesPowNomNeg.y)
        annotation (Line(points={{-2,-10},{-18,-10}}, color={255,0,255}));
      connect(PNetIn, swi.u1) annotation (Line(points={{-120,60},{60,60},{60,38},{
              68,38}},                  color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Battery;

    model Battery2 "Control for the battery energy storage system"
      extends Modelica.Blocks.Icons.Block;
      parameter Modelica.SIunits.Energy EMax(min=0, displayUnit="kWh")
        "Maximum available charge";
      parameter Modelica.SIunits.Power P_nominal(min=0)
        "Nominal power charge/discharge rate";
      parameter Modelica.SIunits.Power PMax(min=0)=10000
        "Maximum power charge/discharge rate";
      parameter Modelica.SIunits.Power PMin(min=0)=100
        "Minimum power charge/discharge rate";
      parameter Modelica.SIunits.Time riseTime=60
        "Rise time of the filter (time to reach 99.6 % of the transition speed)"
        annotation(Dialog(tab="Dynamics", group="Filtered transition speed"));
      Modelica.Blocks.Interfaces.RealInput PNetIn "Net power input"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Modelica.Blocks.Interfaces.RealInput soc "State of charge"
        annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
      Modelica.Blocks.Interfaces.RealOutput P(
        start = 0,
        final quantity="Power",
        final unit="W",
        displayUnit="KW") "Battery power (negative for discharge)"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Continuous.Filter fil(
        analogFilter=Modelica.Blocks.Types.AnalogFilter.CriticalDamping,
        filterType=Modelica.Blocks.Types.FilterType.LowPass,
        order=2,
        f_cut=5/(2*Modelica.Constants.pi*riseTime),
        init=Modelica.Blocks.Types.Init.InitialOutput)
        "Second order filter to approximate battery transitions between charge/off/discharge/off/charge"
        annotation (Placement(transformation(extent={{70,-10},{90,10}})));
      BatteryStage sta(P_nominal=P_nominal) "Stage"
        annotation (Placement(transformation(extent={{-60,-16},{-40,4}})));
      Modelica.Blocks.Math.IntegerToReal intToRea "Integer to real"
        annotation (Placement(transformation(extent={{-20,-16},{0,4}})));
      Modelica.Blocks.Math.Product pro "Product"
        annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Modelica.Blocks.Math.Abs abs "Absolute value"
        annotation (Placement(transformation(extent={{-60,50},{-40,70}})));
      Buildings.Controls.OBC.CDL.Continuous.Limiter lim(final uMax=PMax, final uMin=
           PMin)
        annotation (Placement(transformation(extent={{-20,50},{0,70}})));
    equation
      connect(fil.y, P) annotation (Line(points={{91,0},{110,0}}, color={0,0,127}));
      connect(sta.sta, intToRea.u)
        annotation (Line(points={{-39,-6},{-22,-6}}, color={255,127,0}));
      connect(PNetIn, sta.PNetIn) annotation (Line(points={{-120,60},{-80,60},{-80,0},
              {-62,0}}, color={0,0,127}));
      connect(soc, sta.soc) annotation (Line(points={{-120,-40},{-80,-40},{-80,-10},
              {-62,-10}}, color={0,0,127}));
      connect(intToRea.y, pro.u2)
        annotation (Line(points={{1,-6},{18,-6}}, color={0,0,127}));
      connect(pro.y, fil.u)
        annotation (Line(points={{41,0},{68,0}}, color={0,0,127}));
      connect(PNetIn, abs.u)
        annotation (Line(points={{-120,60},{-62,60}}, color={0,0,127}));
      connect(abs.y, lim.u)
        annotation (Line(points={{-39,60},{-22,60}}, color={0,0,127}));
      connect(lim.y, pro.u1)
        annotation (Line(points={{2,60},{10,60},{10,6},{18,6}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Battery2;

    model BatteryStage "Staging control for battery"
      extends Modelica.Blocks.Icons.Block;
      parameter Real tWai = 60 "Waiting time (s)";
      parameter Modelica.SIunits.Power P_nominal(min=0)
        "Nominal power charge/discharge rate";
      Modelica.Blocks.Interfaces.RealInput soc "State of charge"
        annotation (Placement(transformation(extent={{-140,-60},{-100,-20}})));
      Modelica.Blocks.Interfaces.RealInput PNetIn "Net power input"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Modelica.Blocks.Interfaces.IntegerOutput sta "State"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.StateGraph.Transition chaToOff(condition=PNetIn < P_nominal or soc >=
            0.95)
        "Charge to off" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={10,40})));
      Modelica.StateGraph.Transition offToDis(
        condition=PNetIn <= -P_nominal and soc > 0.05,
        enableTimer=false,                                      waitTime=tWai)
        "Off to discharge" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=-90,
            origin={10,-20})));
      Modelica.StateGraph.Transition disToOff(condition=PNetIn > -P_nominal or soc
             <= 0.05)
        "Discharge to off" annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=-90,
            origin={-70,-20})));
      Modelica.StateGraph.Transition offToCha(
        condition=PNetIn >= P_nominal and soc < 0.95,
        enableTimer=false,                                      waitTime=tWai)
        "Off to charge" annotation (Placement(transformation(
            extent={{10,-10},{-10,10}},
            rotation=-90,
            origin={-70,40})));
      Modelica.StateGraph.StepWithSignal cha "Charge"
        annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
      Modelica.StateGraph.StepWithSignal dis "Discharge"
        annotation (Placement(transformation(extent={{-20,-60},{-40,-40}})));
      Modelica.StateGraph.InitialStep off(nIn=2, nOut=2)
                                          "Off state"
        annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
      Modelica.Blocks.MathInteger.MultiSwitch swi(
        expr={1,-1},
        y_default=0,
        use_pre_as_default=false,
        nu=2) annotation (Placement(transformation(extent={{40,-10},{80,10}})));
      inner Modelica.StateGraph.StateGraphRoot stateGraphRoot
        annotation (Placement(transformation(extent={{60,60},{80,80}})));
    equation
      connect(cha.outPort[1], chaToOff.inPort)
        annotation (Line(points={{-19.5,70},{10,70},{10,44}}, color={0,0,0}));
      connect(off.outPort[1], offToDis.inPort)
        annotation (Line(points={{-19.5,10.25},{10,10.25},{10,-16}},
                                                               color={0,0,0}));
      connect(offToDis.outPort, dis.inPort[1])
        annotation (Line(points={{10,-21.5},{10,-50},{-19,-50}}, color={0,0,0}));
      connect(dis.outPort[1], disToOff.inPort)
        annotation (Line(points={{-40.5,-50},{-70,-50},{-70,-24}}, color={0,0,0}));
      connect(offToCha.outPort, cha.inPort[1])
        annotation (Line(points={{-70,41.5},{-70,70},{-41,70}}, color={0,0,0}));
      connect(disToOff.outPort, off.inPort[1])
        annotation (Line(points={{-70,-18.5},{-70,10.5},{-41,10.5}},
                                                                 color={0,0,0}));
      connect(cha.active, swi.u[1]) annotation (Line(points={{-30,59},{-30,52},{30,52},
              {30,1.5},{40,1.5}}, color={255,0,255}));
      connect(dis.active, swi.u[2]) annotation (Line(points={{-30,-61},{-30,-70},{30,
              -70},{30,-1.5},{40,-1.5}}, color={255,0,255}));
      connect(swi.y, sta)
        annotation (Line(points={{81,0},{110,0}}, color={255,127,0}));
      connect(chaToOff.outPort, off.inPort[2]) annotation (Line(points={{10,38.5},{
              10,26},{-50,26},{-50,9.5},{-41,9.5}}, color={0,0,0}));
      connect(offToCha.inPort, off.outPort[2]) annotation (Line(points={{-70,36},{
              -70,30},{-10,30},{-10,9.75},{-19.5,9.75}}, color={0,0,0}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end BatteryStage;

    model Panel "Control for electrical subsystem"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer n(min=1) "Number of input connectors";
      parameter Real a[n] = fill(1/n, n) "Weighting factors"
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
      Modelica.Blocks.Interfaces.RealVectorInput yIn[n] "Input control signals"
        annotation (Placement(transformation(extent={{-118,-20},{-78,20}})));
      Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Math.MultiSum multiSum(each k=a, nu=n)
        annotation (Placement(transformation(extent={{-56,-6},{-44,6}})));
      Modelica.Blocks.Math.Gain nor(k=sum(a)) "Normalized signal"
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
    equation
      connect(multiSum.y, nor.u)
        annotation (Line(points={{-42.98,0},{-2,0}}, color={0,0,127}));
      connect(nor.y, yOut)
        annotation (Line(points={{21,0},{110,0}}, color={0,0,127}));
      connect(yIn, multiSum.u)
        annotation (Line(points={{-98,0},{-56,0}}, color={0,0,127}));
    end Panel;
  end Controls;

  package Examples "Collection of models that illustrate model use and test models"
    extends Modelica.Icons.ExamplesPackage;

    model Battery "Example model to test and demonstrate the battery"
      extends Modelica.Icons.Example;

      Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid gri(
        f=60,
        V=208,
        phiSou=0)
        "Grid model that provides power to the system"
        annotation (Placement(transformation(extent={{0,40},{20,60}})));
      Modelica.Blocks.Sources.Ramp PNet(
        height=5000,
        duration(displayUnit="min") = 60,
        offset=-2500,
        startTime(displayUnit="min") = 60)
        "Net power"
        annotation (Placement(transformation(extent={{-60,6},{-40,26}})));
      StorageBattery bat(PBat=1000) "Battery"
        annotation (Placement(transformation(extent={{0,0},{20,20}})));
      StorageBattery bat2(
        PBat=1000,
        PMax=1500,
        PMin=500,
        redeclare BICEPS.Electrical.Device.Controls.Battery2    con)
        "Battery"
        annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
    equation
      connect(gri.terminal, bat.terminal)
        annotation (Line(points={{10,40},{10,20.8}},   color={0,120,120}));
      connect(PNet.y, bat.PNetIn)
        annotation (Line(points={{-39,16},{-2,16}},  color={0,0,127}));
      connect(gri.terminal, bat2.terminal) annotation (Line(points={{10,40},{10,30},
              {30,30},{30,-19.2}}, color={0,120,120}));
      connect(PNet.y, bat2.PNetIn) annotation (Line(points={{-39,16},{-20,16},{-20,
              -24},{18,-24}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        __Dymola_Commands(
          file="modelica://BICEPS/Resources/Scripts/Dymola/Electrical/Device/Examples/Battery.mos"
          "Simulate and plot"),
        experiment(StopTime=240,Tolerance=1e-6, __Dymola_Algorithm="Dassl"));
    end Battery;

    model BatteryStage
      "Example model to test and demonstrate the battery staging"
      extends Modelica.Icons.Example;
      Modelica.Blocks.Sources.Ramp PNet(
        height=2500,
        duration(displayUnit="min") = 60,
        offset=-1225,
        startTime(displayUnit="min") = 60) "Net power"
        annotation (Placement(transformation(extent={{-80,26},{-60,46}})));
      Controls.BatteryStage sta(P_nominal=1000) "Stage"
        annotation (Placement(transformation(extent={{-20,20},{0,40}})));
      Modelica.Blocks.Sources.Ramp soc(
        height=1,
        duration(displayUnit="min") = 15,
        offset=0.25,
        startTime(displayUnit="min") = 210) "State of charge"
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
    equation
      connect(PNet.y, sta.PNetIn)
        annotation (Line(points={{-59,36},{-22,36}}, color={0,0,127}));
      connect(soc.y, sta.soc) annotation (Line(points={{-59,0},{-40,0},{-40,26},{-22,
              26}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        __Dymola_Commands(
          file="modelica://BICEPS/Resources/Scripts/Dymola/Electrical/Device/Examples/BatteryStage.mos"
          "Simulate and plot"),
        experiment(StopTime=240,Tolerance=1e-6,__Dymola_Algorithm="Dassl"));
    end BatteryStage;
  end Examples;
end Device;
