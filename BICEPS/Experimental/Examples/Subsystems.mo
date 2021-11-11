within BICEPS.Experimental.Examples;
package Subsystems "Collection of subsystem models"
  extends Modelica.Icons.VariantsPackage;
  model ThermoFluid
    "Thermo-fluid subsystem (secondary) in the renewable supply heat pump example"
    extends Buildings.BaseClasses.BaseIconLow;
    package Medium1 = Buildings.Media.Water "Medium model";
    package Medium2 = Buildings.Media.Water "Medium model";
    parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
    parameter Real kappa(min=Modelica.Constants.small)=10
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
      T0=T0,
      kappa=kappa)
      "Heat pump controller"
      annotation (Placement(transformation(extent={{-60,44},{-40,64}})));
    Sensors.RelativeInternalExergyPotential yHP(
      kappa=kappa,
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
    connect(yHP.yh, conHeaPum.yHeaPum) annotation (Line(points={{-31,13},{-24,
            13},{-24,40},{-70,40},{-70,48},{-62,48}},
                                                  color={0,0,127}));
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
            fillPattern=FillPattern.Solid)}),                      Diagram(
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
