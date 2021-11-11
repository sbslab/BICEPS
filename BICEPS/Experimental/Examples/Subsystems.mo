within BICEPS.Experimental.Examples;
package Subsystems "Collection of subsystem models"
  extends Modelica.Icons.VariantsPackage;
  model Electrical
    parameter Modelica.SIunits.Voltage V_nominal=208
      "Nominal voltage of the line";
    parameter Modelica.SIunits.Frequency f = 60 "Nominal grid frequency";
    parameter Modelica.SIunits.Power PLoa_nominal = 152159
      "Nominal power of a load";
    parameter Modelica.SIunits.Power PWin = PLoa_nominal*4
      "Nominal power of the wind turbine";
    parameter Modelica.SIunits.Power PSun = PLoa_nominal*1.0
      "Nominal power of the PV";
    parameter Modelica.SIunits.Angle lat "Latitude"
      annotation(Evaluate=true,Dialog(group="Orientation"));
    parameter Modelica.SIunits.DensityOfHeatFlowRate W_m2_nominal = 1000
      "Nominal solar power per unit area";
    parameter Real eff_PV = 0.12*0.85*0.9
      "Nominal solar power conversion efficiency (this should consider converion efficiency, area covered, AC/DC losses)";
    parameter Modelica.SIunits.Area A_PV = PSun/eff_PV/W_m2_nominal
      "Nominal area of a P installation";
    parameter Modelica.SIunits.Power PBat = 1000
      "Nominal power charge/discharge rate of the battery";
      parameter Modelica.SIunits.Energy EBatMax = 10000
      "Maximum energy capacity of the battery";
    Modelica.Blocks.Interfaces.RealInput PHeaPum "Heat pump power"
      annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Interfaces.RealOutput yEle
      "Electrical subsystem control signal "
      annotation (Placement(transformation(extent={{100,-10},{120,10}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linGri(
      l=1500,
      V_nominal=V_nominal)
      "Grid power line"
      annotation (Placement(transformation(extent={{-60,60},{-40,80}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Storage.Battery bat(
      redeclare package PhaseSystem =
        Buildings.Electrical.PhaseSystems.ThreePhase_dq,
      SOC_start=0.5,
      EMax=EBatMax,
      V_nominal=V_nominal)
      annotation (Placement(transformation(extent={{-60,-40},{-80,-60}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Loads.Inductive loa(
      linearized=false,
      mode=Buildings.Electrical.Types.Load.VariableZ_P_input)
      annotation (Placement(transformation(extent={{-60,-10},{-80,10}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
      annotation (Placement(transformation(extent={{-120,60},{-100,80}}),
                                iconTransformation(extent={{-10,-110},{10,-90}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      "Weather data bus"
      annotation (Placement(transformation(extent={{-20,80},{20,120}}),
        iconTransformation(extent={{-10,90},{10,110}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linWin(
      l=300,
      V_nominal=V_nominal)
      "Wind electrical line"
      annotation (Placement(transformation(extent={{30,60},{10,80}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linPV(
      l=300,
      V_nominal=V_nominal)
      "PV electrical line"
      annotation (Placement(transformation(extent={{30,20},{10,40}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linHP(
      l=10,
      P_nominal=PLoa_nominal,
      V_nominal=V_nominal)
      "Heat pump electrical line"
      annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Lines.Line linBat(
      l=10,
      V_nominal=V_nominal)
      "Battery electrical line"
      annotation (Placement(transformation(extent={{-30,-60},{-10,-40}})));
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
    Sensors.RelativeElectricalExergyPotential yLoa(v0=V_nominal)
      "Control signal load"
      annotation (Placement(transformation(extent={{-50,10},{-30,30}})));
    Sensors.RelativeElectricalExergyPotential yBat(v0=V_nominal)
      "Control signal battery"
      annotation (Placement(transformation(extent={{-50,-40},{-30,-20}})));
    Sensors.RelativeElectricalExergyPotential yPV(v0=V_nominal)
      "Control signal pv"
      annotation (Placement(transformation(extent={{50,20},{30,0}})));
    Sensors.RelativeElectricalExergyPotential yWin(v0=V_nominal)
      "Control signal wind turbine"
      annotation (Placement(transformation(extent={{50,60},{30,40}})));
    Sensors.RelativeElectricalExergyPotential yGri(v0=V_nominal)
      "Control signal grid"
      annotation (Placement(transformation(extent={{-90,60},{-70,40}})));
    Controls.SubsystemElectrical conSubEle(n=5)
      annotation (Placement(transformation(extent={{20,-40},{40,-20}})));
    Controls.Battery conBat(EMax=EBatMax, P_nominal=PBat)
      "Battery controller"
      annotation (Placement(transformation(extent={{-40,-90},{-60,-70}})));
  equation
    connect(PHeaPum, loa.Pow)
      annotation (Line(points={{-120,0},{-80,0}}, color={0,0,127}));
    connect(terminal, linGri.terminal_n)
      annotation (Line(points={{-110,70},{-60,70}}, color={0,120,120}));
    connect(linGri.terminal_p, linWin.terminal_p)
      annotation (Line(points={{-40,70},{10,70}}, color={0,120,120}));
    connect(linGri.terminal_p, linBat.terminal_p) annotation (Line(points={{-40,70},
            {0,70},{0,-50},{-10,-50}}, color={0,120,120}));
    connect(linBat.terminal_n, bat.terminal)
      annotation (Line(points={{-30,-50},{-60,-50}}, color={0,120,120}));
    connect(linHP.terminal_n, loa.terminal)
      annotation (Line(points={{-30,0},{-60,0}}, color={0,120,120}));
    connect(linHP.terminal_p, linBat.terminal_p) annotation (Line(points={{-10,0},
            {0,0},{0,-50},{-10,-50}}, color={0,120,120}));
    connect(linPV.terminal_p, linBat.terminal_p) annotation (Line(points={{10,30},
            {0,30},{0,-50},{-10,-50}}, color={0,120,120}));
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
    connect(yLoa.terminal, loa.terminal)
      annotation (Line(points={{-40,10},{-40,0},{-60,0}}, color={0,120,120}));
    connect(yBat.terminal, bat.terminal) annotation (Line(points={{-40,-40},{-40,-50},
            {-60,-50}}, color={0,120,120}));
    connect(yPV.terminal, pv.terminal)
      annotation (Line(points={{40,20},{40,30},{62,30}}, color={0,120,120}));
    connect(yWin.terminal, winTur.terminal)
      annotation (Line(points={{40,60},{40,70},{60,70}}, color={0,120,120}));
    connect(yGri.terminal, linGri.terminal_n)
      annotation (Line(points={{-80,60},{-80,70},{-60,70}}, color={0,120,120}));
    connect(yBat.y, conSubEle.yIn) annotation (Line(points={{-29,-23},{6,-23},{6,-30},
            {20.2,-30}}, color={0,0,127}));
    connect(yPV.y, conSubEle.yIn) annotation (Line(points={{29,3},{6,3},{6,-30},{20.2,
            -30}}, color={0,0,127}));
    connect(yWin.y, conSubEle.yIn) annotation (Line(points={{29,43},{6,43},{6,-30},
            {20.2,-30}}, color={0,0,127}));
    connect(yGri.y, conSubEle.yIn) annotation (Line(points={{-69,43},{6,43},{6,-30},
            {20.2,-30}}, color={0,0,127}));
    connect(yLoa.y, conSubEle.yIn) annotation (Line(points={{-29,27},{6,27},{6,-30},
            {20.2,-30}}, color={0,0,127}));
    connect(conSubEle.yOut, yEle) annotation (Line(points={{41,-30},{90,-30},{90,0},
            {110,0}}, color={0,0,127}));
    connect(conSubEle.yOut, conBat.yEle) annotation (Line(points={{41,-30},{50,-30},
            {50,-74},{-38,-74}}, color={0,0,127}));
    connect(bat.SOC, conBat.soc) annotation (Line(points={{-81,-56},{-90,-56},{-90,
            -96},{-30,-96},{-30,-86},{-38,-86}}, color={0,0,127}));
    connect(conBat.P, bat.P)
      annotation (Line(points={{-61,-80},{-70,-80},{-70,-60}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
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
          smooth=Smooth.None)}), Diagram(coordinateSystem(preserveAspectRatio=false)));
  end Electrical;

  model ThermoFluid
    "Thermo-fluid subsystem (secondary) in the renewable supply heat pump example"
    extends Buildings.BaseClasses.BaseIconLow;
    package Medium1 = Buildings.Media.Water "Medium model";
    package Medium2 = Buildings.Media.Water "Medium model";
    parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
    parameter Real k(min=Modelica.Constants.small)=10
      "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    parameter Modelica.SIunits.TemperatureDifference dTEva_nominal=-5
      "Temperature difference evaporator inlet-outlet";
    parameter Modelica.SIunits.TemperatureDifference dTCon_nominal=10
      "Temperature difference condenser outlet-inlet";
    parameter Modelica.SIunits.HeatFlowRate QCon_flow_nominal = 100E3
      "Evaporator heat flow rate";
    parameter Modelica.SIunits.MassFlowRate m1_flow_nominal=
      QCon_flow_nominal/dTCon_nominal/4200 "Nominal mass flow rate at condenser";
    final parameter Modelica.SIunits.SpecificHeatCapacity cp1_default=
      Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
        Medium1.p_default,
        Medium1.T_default,
        Medium1.X_default))
      "Specific heat capacity of medium 1 at default medium state";
    final parameter Modelica.SIunits.SpecificHeatCapacity cp2_default=
      Medium2.specificHeatCapacityCp(Medium2.setState_pTX(
        Medium2.p_default,
        Medium2.T_default,
        Medium2.X_default))
      "Specific heat capacity of medium 2 at default medium state";
    Buildings.Fluid.HeatPumps.Carnot_TCon heaPum(
      redeclare package Medium1 = Medium1,
      redeclare package Medium2 = Medium2,
      dTEva_nominal=dTEva_nominal,
      dTCon_nominal=dTCon_nominal,
      m1_flow_nominal=m1_flow_nominal,
      show_T=true,
      allowFlowReversal1=false,
      allowFlowReversal2=false,
      use_eta_Carnot_nominal=true,
      etaCarnot_nominal=0.3,
      QCon_flow_nominal=QCon_flow_nominal,
      dp1_nominal=6000,
      dp2_nominal=6000) "Heat pump model"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}})));
    Buildings.Fluid.Sources.MassFlowSource_T souCon(
      redeclare package Medium = Medium1,
      use_T_in=false,
      m_flow=m1_flow_nominal,
      T=293.15,
      nPorts=1) "Condenser source"
      annotation (Placement(transformation(extent={{-80,-4},{-60,16}})));
    Buildings.Fluid.Sources.MassFlowSource_T evaSou(
      redeclare package Medium = Medium2,
      use_m_flow_in=true,
      use_T_in=true,
      nPorts=1)
      "Evaporator source"
      annotation (Placement(transformation(extent={{50,-70},{30,-50}})));
    Buildings.Fluid.Sources.Boundary_pT conSin(
      redeclare package Medium = Medium1, nPorts=1)
      "Condenser sink"
       annotation (Placement(transformation(extent={{10,-10},{-10,10}}, origin={80,20})));
    Buildings.Fluid.Sources.Boundary_pT evaSin(
      redeclare package Medium = Medium2,
      nPorts=1)
      "Evaporator sink"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}}, origin={-70,-60})));
    Modelica.Blocks.Interfaces.RealOutput PHeaPum "Heat pump power"
      annotation (Placement(transformation(extent={{100,50},{120,70}})));
    Modelica.Blocks.Interfaces.RealInput yEle
      "Relative exergetic potential of electrical subsystem"
      annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
    Modelica.Blocks.Sources.Constant TEvaIn(k=273.15 + 15)
      "Inlet temperature to the evaporator"
      annotation (Placement(transformation(extent={{90,-90},{70,-70}})));
    Modelica.Blocks.Math.Gain mEva_flow(k=1/cp2_default/dTEva_nominal)
      "Evaporator mass flow rate"
      annotation (Placement(transformation(extent={{60,-20},{80,0}})));
    Controls.HeatPump conHeaPum(
      TMin=TMin,
      TMax=TMax,
      T0=T0)
      "Heat pump controller"
      annotation (Placement(transformation(extent={{-60,44},{-40,64}})));
    Sensors.RelativeInternalExergyPotential yHP(
      k=k,
      redeclare package Medium = Medium1,
      m_flow_nominal=m1_flow_nominal)
      annotation (Placement(transformation(extent={{-52,-4},{-32,16}})));
  equation
    connect(evaSou.ports[1], heaPum.port_a2) annotation (Line(
        points={{30,-60},{20,-60},{20,-6},{10,-6}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(evaSin.ports[1], heaPum.port_b2) annotation (Line(
        points={{-60,-60},{-20,-60},{-20,-6},{-10,-6}},
        color={0,127,255},
        smooth=Smooth.None));
    connect(TEvaIn.y, evaSou.T_in)
      annotation (Line(points={{69,-80},{60,-80},{60,-56},{52,-56}},
                                                   color={0,0,127}));
    connect(heaPum.P, PHeaPum) annotation (Line(points={{11,0},{20,0},{20,60},{110,
            60}}, color={0,0,127}));
    connect(mEva_flow.y, evaSou.m_flow_in) annotation (Line(points={{81,-10},{89,-10},
            {89,-52},{52,-52}}, color={0,0,127}));
    connect(heaPum.QEva_flow, mEva_flow.u) annotation (Line(points={{11,-9},{10,-9},
            {10,-10},{58,-10}}, color={0,0,127}));
    connect(conHeaPum.TSet, heaPum.TSet) annotation (Line(points={{-39,54},{-20,54},
            {-20,9},{-12,9}}, color={0,0,127}));
    connect(conHeaPum.yEle, yEle)
      annotation (Line(points={{-62,60},{-120,60}}, color={0,0,127}));
    connect(yHP.y, conHeaPum.yHeaPum) annotation (Line(points={{-31,13},{-24,13},{
            -24,40},{-70,40},{-70,48},{-62,48}}, color={0,0,127}));
    connect(souCon.ports[1], yHP.port_a)
      annotation (Line(points={{-60,6},{-52,6}}, color={0,127,255}));
    connect(yHP.port_b, heaPum.port_a1)
      annotation (Line(points={{-32,6},{-10,6}}, color={0,127,255}));
    connect(heaPum.port_b1, conSin.ports[1]) annotation (Line(points={{10,6},{
            50,6},{50,20},{70,20}}, color={0,127,255}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-80,80},{80,-80}},
            lineColor={0,0,0},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-18,50},{22,42}},
            fillPattern=FillPattern.Solid),
          Line(points={{2,42},{2,-10}}),
          Polygon(points={{-70,26},{68,-44},{68,26},{2,-10},{-70,-42},{-70,26}})}),
                                                                   Diagram(
          coordinateSystem(preserveAspectRatio=false)));
  end ThermoFluid;

  package Examples
    model ThermoFluid "Example model for the thermofluid subsystem"
      extends Modelica.Icons.Example;
      BICEPS.Experimental.Examples.Subsystems.ThermoFluid staTF(
        TMin=303.15,
        TMax=323.15,
        T0=313.15) annotation (Placement(transformation(extent={{0,0},{20,20}})));
      Modelica.Blocks.Sources.Constant conSta(k=0) "Constant ideal state"
        annotation (Placement(transformation(extent={{-60,46},{-40,66}})));
      Modelica.Blocks.Sources.Sine staPul(freqHz=1) "Stable pulse"
        annotation (Placement(transformation(extent={{-60,6},{-40,26}})));
      BICEPS.Experimental.Examples.Subsystems.ThermoFluid conTF(
        TMin=303.15,
        TMax=323.15,
        T0=313.15) annotation (Placement(transformation(extent={{0,40},{20,60}})));
      BICEPS.Experimental.Examples.Subsystems.ThermoFluid unStaTF(
        TMin=303.15,
        TMax=323.15,
        T0=313.15) annotation (Placement(transformation(extent={{0,-40},{20,-20}})));
      Modelica.Blocks.Sources.Sine unStaPul(amplitude=2, freqHz=1) "Unstable pulse"
        annotation (Placement(transformation(extent={{-60,-34},{-40,-14}})));
    equation
      connect(staPul.y, staTF.yEle)
        annotation (Line(points={{-39,16},{-2,16}}, color={0,0,127}));
      connect(conSta.y, conTF.yEle)
        annotation (Line(points={{-39,56},{-2,56}}, color={0,0,127}));
      connect(unStaPul.y, unStaTF.yEle) annotation (Line(points={{-39,-24},{-20,-24},
              {-20,-24},{-2,-24}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
       __Dymola_Commands(
          file="modelica://BICEPS/Resources/Scripts/Dymola/Experimental/Examples/Subsystems/Examples/ThermoFluid.mos"
          "Simulate and plot"),
        experiment(
          StopTime=1,
          Tolerance=1e-06),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
    end ThermoFluid;
  end Examples;
end Subsystems;
