within BICEPS.Fluid;
package Device "Package with device-level models"
  model FanCoilWithDistributionPump
    "Fan coil unit model with distribution pump"
    extends Buildings.BaseClasses.BaseIconLow;
    replaceable package Medium1=Buildings.Media.Water
      constrainedby Modelica.Media.Interfaces.PartialMedium
      "Medium in the building distribution system";
    replaceable package Medium2=Buildings.Media.Air
      constrainedby Modelica.Media.Interfaces.PartialMedium
      "Load side medium";
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    // Diagnostics
     parameter Boolean show_T1 = false
      "= true, if actual temperature at port is computed"
      annotation (
        Dialog(tab="Advanced", group="Diagnostics"),
        HideResult=true);
     parameter Boolean show_T2 = false
      "= true, if actual temperature at port is computed"
      annotation (
        Dialog(tab="Advanced", group="Diagnostics"),
        HideResult=true);
    parameter Boolean allowFlowReversal1=false
      "Set to true to allow flow reversal in building distribution system"
      annotation (Dialog(tab="Assumptions"),Evaluate=true);
    parameter Boolean allowFlowReversal2=true
      "Set to true to allow flow reversal on the load side"
      annotation (Dialog(tab="Assumptions"),Evaluate=true);
    parameter Modelica.SIunits.MassFlowRate m1_flow_nominal(
      min=0)=0
      "Building distribution side mass flow rate at nominal conditions in heating mode"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.MassFlowRate m2_flow_nominal(
      min=0)=0
      "Load side mass flow rate at nominal conditions in heating mode"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.PressureDifference dp1_nominal(
      final min=0,
      displayUnit="Pa")
      "Pressure drop at nominal conditions on building distribution side"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.PressureDifference dp2_nominal(
      final min=0,
      displayUnit="Pa")
      "Pressure drop at nominal conditions on load side"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.HeatFlowRate QHea_flow_nominal(
      min=0)=0
      "Nominal heating capacity (>=0)"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Temperature TMin=288.15
      "Minimimum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature TMax=298.15
      "Maximum desired threshold for independent variable";
    parameter Modelica.SIunits.Temperature T0=293.15
      "Nominal value for independent variable. Fixed setpoint if not biomimetic control.";
    // AHRI 440 Standard Heating
    parameter Modelica.SIunits.Temperature T_aHeaWat_nominal=273.15 + 60
      "Heating water inlet temperature at nominal conditions"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Temperature T_bHeaWat_nominal(
      min=273.15,
      displayUnit="degC")=T_aHeaWat_nominal-22.2
      "Heating water outlet temperature at nominal conditions"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Temperature T_aLoaHea_nominal=273.15 + 21.1
      "Load side inlet temperature at nominal conditions in heating mode"
      annotation (Dialog(group="Nominal condition"));
    Medium1.ThermodynamicState sta_a1=
        Medium1.setState_phX(port_a1.p,
                            noEvent(actualStream(port_a1.h_outflow)),
                            noEvent(actualStream(port_a1.Xi_outflow))) if
           show_T1 "Medium properties in port_a1";

    Medium1.ThermodynamicState sta_b1=
        Medium1.setState_phX(port_b1.p,
                            noEvent(actualStream(port_b1.h_outflow)),
                            noEvent(actualStream(port_b1.Xi_outflow))) if
            show_T1 "Medium properties in port_b1";
    Medium2.ThermodynamicState sta_a2=
        Medium2.setState_phX(port_a2.p,
                            noEvent(actualStream(port_a2.h_outflow)),
                            noEvent(actualStream(port_a2.Xi_outflow))) if
           show_T2 "Medium properties in port_a1";

    Medium2.ThermodynamicState sta_b2=
        Medium2.setState_phX(port_b1.p,
                            noEvent(actualStream(port_b2.h_outflow)),
                            noEvent(actualStream(port_b2.Xi_outflow))) if
            show_T2 "Medium properties in port_b2";
    replaceable parameter Buildings.Fluid.Movers.Data.Generic per(
      pressure(
        V_flow=m1_flow_nominal/rho1_default .* {0,1,2},
        dp=dp1_nominal .* {1.5,1,0.5}),
      motorCooledByFluid=false)
      constrainedby Buildings.Fluid.Movers.Data.Generic
      "Record with performance data"
      annotation (choicesAllMatching=true,
        Placement(transformation(extent={{60,-40},{80,-20}})));
    Buildings.Fluid.FixedResistances.PressureDrop resDis(
      redeclare final package Medium = Medium1,
      final m_flow_nominal=m1_flow_nominal,
      final dp_nominal=dp1_nominal) "Distribution resistence pipe"
      annotation (Placement(transformation(extent={{-50,-70},{-30,-50}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b1(
      p(start=Medium1.p_default),
      redeclare final package Medium = Medium1,
      m_flow(max=if allowFlowReversal1 then +Modelica.Constants.inf else 0),
      h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
      "Fluid connector b (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{110,-70},{90,-50}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a1(
      p(start=Medium1.p_default),
      redeclare final package Medium = Medium1,
      m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
      h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
      "Fluid connector a (positive design flow direction is from port_a to port_b)"
      annotation (Placement(transformation(extent={{-110,-70},{-90,-50}})));
    Buildings.Fluid.HeatExchangers.DryCoilEffectivenessNTU hex(
      redeclare final package Medium1 = Medium1,
      redeclare final package Medium2 = Medium2,
      final configuration=Buildings.Fluid.Types.HeatExchangerConfiguration.CounterFlow,
      final m1_flow_nominal=m1_flow_nominal,
      final m2_flow_nominal=m2_flow_nominal,
      final dp1_nominal=0,
      final dp2_nominal=0,
      final Q_flow_nominal=QHea_flow_nominal,
      final T_a1_nominal=T_aHeaWat_nominal,
      final T_a2_nominal=T_aLoaHea_nominal,
      final allowFlowReversal1=allowFlowReversal1,
      final allowFlowReversal2=allowFlowReversal2)
      "Heating coil"
      annotation (Placement(transformation(extent={{-20,0},{0,-20}})));
    Buildings.Fluid.Movers.FlowControlled_m_flow fan(
      redeclare final package Medium = Medium2,
      final allowFlowReversal=allowFlowReversal2,
      final m_flow_nominal=m2_flow_nominal,
      redeclare final Buildings.Fluid.Movers.Data.Generic per,
      nominalValuesDefineDefaultPressureCurve=true,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial,
      final dp_nominal=dp2_nominal)
      "Fan"
      annotation (Placement(transformation(extent={{30,30},{10,50}})));
    Buildings.Fluid.FixedResistances.PressureDrop resLoa(
      redeclare final package Medium = Medium2,
      final m_flow_nominal=m2_flow_nominal,
      final dp_nominal=dp2_nominal)
      "Load side pressure drop"
      annotation (Placement(transformation(extent={{60,30},{40,50}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a2(
      redeclare final package Medium = Medium2,
      p(start=Medium2.p_default),
      m_flow(min=if allowFlowReversal2 then -Modelica.Constants.inf else 0),
      h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
      "Fluid stream inlet port on the load side"
      annotation (Placement(transformation(extent={{90,30},{110,50}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b2(
      redeclare final package Medium = Medium2,
      p(start=Medium2.p_default),
      m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
      h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
      "Fluid stream outlet port on the load side"
      annotation (Placement(transformation(extent={{-90,30},{-110,50}})));
    Buildings.Fluid.Sources.Boundary_pT pRefFan(redeclare package Medium =
          Medium2, nPorts=1) "Reference pressure"
      annotation (Placement(transformation(extent={{60,0},{40,20}})));
    Modelica.Blocks.Interfaces.RealInput y if biomimeticControl "Control signal"
      annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
    Modelica.Blocks.Math.Gain m2Set_flow(k=m2_flow_nominal)
      "Mass flow setpoint for the fan"
      annotation (Placement(transformation(extent={{-20,70},{0,90}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTem(redeclare package Medium =
          Medium2, m_flow_nominal=m2_flow_nominal) annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=0,
          origin={80,40})));
    Controls.Pump2 con(
      biomimeticControl=biomimeticControl,
      TMin=TMin,
      TMax=TMax,
      T0=T0) "Pump/fan control"
      annotation (Placement(transformation(extent={{-60,70},{-40,90}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput PFan(final unit="W")
      "Fan power"
      annotation (Placement(transformation(extent={{100,90},{120,110}}),
      iconTransformation(extent={{100,40},{140,80}})));
    Modelica.Blocks.Sources.Constant TSetSta(k=T0) if not biomimeticControl
      "Static setpoint temperature if not biomimetic control"
      annotation (Placement(transformation(extent={{-100,90},{-80,110}})));
  protected
    parameter Medium1.ThermodynamicState sta1_default=Medium1.setState_pTX(
      T=Medium1.T_default,
      p=Medium1.p_default,
      X=Medium1.X_default);
    parameter Modelica.SIunits.Density rho1_default=Medium1.density(
      sta1_default)
      "Density, used to compute fluid volume";
  equation
    connect(port_b2, hex.port_b2) annotation (Line(points={{-100,40},{-24,40},{
            -24,-4},{-20,-4}},
                           color={0,127,255}));
    connect(resLoa.port_b, fan.port_a)
      annotation (Line(points={{40,40},{30,40}}, color={0,127,255}));
    connect(resDis.port_b, hex.port_a1) annotation (Line(points={{-30,-60},{-24,
            -60},{-24,-16},{-20,-16}},
                              color={0,127,255}));
    connect(hex.port_b1, port_b1) annotation (Line(points={{0,-16},{4,-16},{4,-60},
            {100,-60}},         color={0,127,255}));
    connect(pRefFan.ports[1], fan.port_a) annotation (Line(points={{40,10},{34,10},
            {34,40},{30,40}}, color={0,127,255}));
    connect(m2Set_flow.y, fan.m_flow_in)
      annotation (Line(points={{1,80},{20,80},{20,52}}, color={0,0,127}));
    connect(fan.port_b, hex.port_a2) annotation (Line(points={{10,40},{4,40},{4,
            -4},{0,-4}}, color={0,127,255}));
    connect(port_a2, senTem.port_a)
      annotation (Line(points={{100,40},{90,40}}, color={0,127,255}));
    connect(senTem.port_b, resLoa.port_a)
      annotation (Line(points={{70,40},{60,40}}, color={0,127,255}));
    connect(y, con.y)
      annotation (Line(points={{-120,80},{-62,80}}, color={0,0,127}));
    connect(con.yOut, m2Set_flow.u)
      annotation (Line(points={{-39,80},{-22,80}}, color={0,0,127}));
    connect(senTem.T, con.TMea) annotation (Line(points={{80,51},{80,60},{-70,60},
            {-70,74},{-62,74}}, color={0,0,127}));
    connect(port_a1, resDis.port_a)
      annotation (Line(points={{-100,-60},{-50,-60}}, color={0,127,255}));
    connect(TSetSta.y, con.TSetSta) annotation (Line(points={{-79,100},{-70,100},
            {-70,86},{-62,86}}, color={0,0,127}));
    connect(PFan, fan.P) annotation (Line(points={{110,100},{4,100},{4,49},{9,49}},
          color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-100,-66},{100,-54}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-100,34},{100,46}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-72,60},{72,-80}},
            lineColor={27,0,55},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid)}),                      Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{100,120}})));
  end FanCoilWithDistributionPump;

  model HeatPump "Heat pump model"
    extends Buildings.BaseClasses.BaseIconLow;
    replaceable package Medium1=Modelica.Media.Interfaces.PartialMedium
      "Medium model on condenser side"
      annotation (choices(choice(redeclare package Medium=Buildings.Media.Water
                                                                                "Water"),
      choice(redeclare package Medium =
        Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=293.15,X_a=0.40)
      "Propylene glycol water, 40% mass fraction")));
    replaceable package Medium2=Modelica.Media.Interfaces.PartialMedium
      "Medium model on evaporator side"
      annotation (choices(choice(redeclare package Medium=Buildings.Media.Water
                                                                                "Water"),
      choice(redeclare package Medium =
        Buildings.Media.Antifreeze.PropyleneGlycolWater (property_T=293.15,X_a=0.40)
      "Propylene glycol water, 40% mass fraction")));
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Real COP_nominal(final unit="1")
      "Heat pump COP"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Temperature TCon_nominal
      "Condenser outlet temperature used to compute COP_nominal"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Temperature TEva_nominal
      "Evaporator outlet temperature used to compute COP_nominal"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Temperature THeaWatSup_nominal=313.15
      "Heating water supply temperature"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.HeatFlowRate Q1_flow_nominal(min=0)
      "Heating heat flow rate"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.TemperatureDifference dT1_nominal(
      final min=0) = 5 "Temperature difference condenser outlet-inlet"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.TemperatureDifference dT2_nominal(
      final max=0) = -5 "Temperature difference evaporator outlet-inlet"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Pressure dp1_nominal(displayUnit="Pa")
      "Pressure difference over condenser"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Pressure dp2_nominal(displayUnit="Pa")
      "Pressure difference over evaporator"
      annotation (Dialog(group="Nominal condition"));
     parameter Boolean show_T1 = false
      "= true, if actual temperature at port is computed"
      annotation (
        Dialog(tab="Advanced", group="Diagnostics"),
        HideResult=true);
     parameter Boolean show_T2 = false
      "= true, if actual temperature at port is computed"
      annotation (
        Dialog(tab="Advanced", group="Diagnostics"),
        HideResult=true);
    parameter Boolean allowFlowReversal1=false
      "Set to true to allow flow reversal on condenser side"
      annotation (Dialog(tab="Assumptions"), Evaluate=true);
    parameter Boolean allowFlowReversal2=false
      "Set to true to allow flow reversal on evaporator side"
      annotation (Dialog(tab="Assumptions"), Evaluate=true);
    final parameter Modelica.SIunits.MassFlowRate m1_flow_nominal(min=0)=
      heaPum.m1_flow_nominal
      "Mass flow rate on condenser side"
      annotation (Dialog(group="Nominal condition"));
    final parameter Modelica.SIunits.MassFlowRate m2_flow_nominal(min=0)=
      heaPum.m2_flow_nominal
      "Mass flow rate on evaporator side"
      annotation (Dialog(group="Nominal condition"));
    constant Modelica.SIunits.SpecificHeatCapacity cp1_default=
      Medium1.specificHeatCapacityCp(Medium1.setState_pTX(
        p = Medium1.p_default,
        T = Medium1.T_default))
      "Specific heat capacity of the fluid";
    Medium1.ThermodynamicState sta_a1=
        Medium1.setState_phX(port_a1.p,
                            noEvent(actualStream(port_a1.h_outflow)),
                            noEvent(actualStream(port_a1.Xi_outflow))) if
           show_T1 "Medium properties in port_a1";

    Medium1.ThermodynamicState sta_b1=
        Medium1.setState_phX(port_b1.p,
                            noEvent(actualStream(port_b1.h_outflow)),
                            noEvent(actualStream(port_b1.Xi_outflow))) if
            show_T1 "Medium properties in port_b1";
    Medium2.ThermodynamicState sta_a2=
        Medium2.setState_phX(port_a2.p,
                            noEvent(actualStream(port_a2.h_outflow)),
                            noEvent(actualStream(port_a2.Xi_outflow))) if
           show_T2 "Medium properties in port_a1";

    Medium2.ThermodynamicState sta_b2=
        Medium2.setState_phX(port_b1.p,
                            noEvent(actualStream(port_b2.h_outflow)),
                            noEvent(actualStream(port_b2.Xi_outflow))) if
            show_T2 "Medium properties in port_b2";
    Buildings.Fluid.HeatPumps.Carnot_TCon heaPum(
      redeclare final package Medium1 = Medium1,
      redeclare final package Medium2 = Medium2,
      show_T=show_T1,
      final dTEva_nominal=dT2_nominal,
      final dTCon_nominal=dT1_nominal,
      final TCon_nominal=TCon_nominal,
      final TEva_nominal=TEva_nominal,
      final allowFlowReversal1=allowFlowReversal1,
      final allowFlowReversal2=allowFlowReversal2,
      final use_eta_Carnot_nominal=false,
      final COP_nominal=COP_nominal,
      final QCon_flow_nominal=Q1_flow_nominal,
      final dp1_nominal=dp1_nominal,
      final dp2_nominal=dp2_nominal,
      T1_start=TCon_nominal,
      T2_start=TEva_nominal,
      energyDynamics=Modelica.Fluid.Types.Dynamics.FixedInitial)
                                     "Heat pump (index 1 for condenser side)"
      annotation (Placement(transformation(extent={{10,-40},{30,-20}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTConEnt(
      redeclare final package Medium = Medium1,
      final allowFlowReversal=allowFlowReversal1,
      final m_flow_nominal=m1_flow_nominal)
      "Condenser water entering temperature" annotation (Placement(transformation(
          extent={{-80,10},{-60,30}})));
    Buildings.Experimental.DHC.EnergyTransferStations.BaseClasses.Pump_m_flow pumCon(
      redeclare final package Medium = Medium1,
      energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      final m_flow_nominal=m1_flow_nominal,
      final allowFlowReversal=allowFlowReversal1,
      show_T=show_T1,
      use_inputFilter=true)
      "Heat pump condenser water pump"
      annotation (Placement(transformation(extent={{-40,10},{-20,30}})));
    Buildings.Experimental.DHC.EnergyTransferStations.BaseClasses.Pump_m_flow pumEva(
      redeclare final package Medium = Medium2,
      energyDynamics=Modelica.Fluid.Types.Dynamics.DynamicFreeInitial,
      final m_flow_nominal=m2_flow_nominal,
      final allowFlowReversal=allowFlowReversal2,
      show_T=show_T2,
      use_inputFilter=true)
      "Heat pump evaporator water pump"
      annotation (Placement(transformation(extent={{120,-110},{100,-90}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput PPum(final unit="W")
      "Pump power"
      annotation (Placement(transformation(extent={{140,60},{160,80}}),
      iconTransformation(extent={{100,40},{140,80}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput PHea(final unit="W")
      "Heat pump power"
      annotation (Placement(transformation(extent={{140,100},{160,120}}),
      iconTransformation(extent={{100,80},{140,120}})));
    Buildings.Controls.OBC.CDL.Continuous.Add addPum "Adder"
      annotation (Placement(transformation(extent={{110,60},{130,80}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b2(
      redeclare final package Medium = Medium2,
      m_flow(max=if allowFlowReversal2 then +Modelica.Constants.inf else 0),
      h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
      "Fluid port for leaving evaporator water"
      annotation (Placement(
          transformation(extent={{-150,-110},{-130,-90}}),
          iconTransformation(extent={{-110,-90},{-90,-70}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a2(
      redeclare final package Medium = Medium2,
      m_flow(min=if allowFlowReversal2 then -Modelica.Constants.inf else 0),
      h_outflow(start=Medium2.h_default, nominal=Medium2.h_default))
      "Fluid port for entering evaporator water"
      annotation (Placement(
          transformation(extent={{130,-110},{150,-90}}),
          iconTransformation(extent={{90,-90},{110,-70}})));
    Modelica.Fluid.Interfaces.FluidPort_a port_a1(
      redeclare final package Medium = Medium1,
      m_flow(min=if allowFlowReversal1 then -Modelica.Constants.inf else 0),
      h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
      "Fluid port for entering condenser water"
      annotation (Placement(
          transformation(extent={{-150,10},{-130,30}}),
          iconTransformation(extent={{-110,10},{-90,30}})));
    Modelica.Fluid.Interfaces.FluidPort_b port_b1(
      redeclare final package Medium = Medium1,
      m_flow(max=if allowFlowReversal1 then +Modelica.Constants.inf else 0),
      h_outflow(start=Medium1.h_default, nominal=Medium1.h_default))
      "Fluid port for leaving condenser water"
      annotation (Placement(
          transformation(extent={{130,10},{150,30}}),
          iconTransformation(extent={{90,10},{110,30}})));
    Controls.HeatPump conHeaPum(
      biomimeticControl=biomimeticControl,
      TMin=conHeaPum.T0 - 10,
      TMax=conHeaPum.T0 + 10,
      T0=323.15,
      riseTime=60,
      THeaWatSup_nominal=THeaWatSup_nominal) "Heat pump control"
      annotation (Placement(transformation(extent={{-80,-30},{-60,-50}})));
    Buildings.Controls.OBC.CDL.Continuous.GreaterThreshold staPum[2](
      y(each start=false),
      t=1e-2 .* {m1_flow_nominal,m2_flow_nominal},
      h=0.5e-2 .* {m1_flow_nominal,m2_flow_nominal})
      "Pump return status"
      annotation (Placement(transformation(extent={{-20,-90},{-40,-70}})));
    Buildings.Controls.OBC.CDL.Logical.And ena
      "Enable heat pump if pump return status on"
      annotation (Placement(transformation(extent={{-62,-90},{-82,-70}})));
    Modelica.Blocks.Interfaces.BooleanInput u
      annotation (Placement(transformation(extent={{-180,100},{-140,140}}),
          iconTransformation(extent={{-120,90},{-100,110}})));
    Buildings.Fluid.Sensors.TemperatureTwoPort senTConLea(
      redeclare final package Medium = Medium1,
      final allowFlowReversal=allowFlowReversal1,
      final m_flow_nominal=m1_flow_nominal) "Leaving condenser water temperature"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=0,
          origin={72,20})));
    Buildings.Fluid.Sensors.MassFlowRate senMasFloHeaWat(
      redeclare final package Medium = Medium1,
      final allowFlowReversal=allowFlowReversal1)
      "Heating water mass flow rate"
      annotation (Placement(transformation(extent={{-120,10},{-100,30}})));
    Buildings.Controls.OBC.CDL.Continuous.Add dTHHW(k1=-1, k2=+1)
      "Heating hot water DeltaT"
      annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
    Buildings.Controls.OBC.CDL.Continuous.Gain capFloHHW(final k=cp1_default)
      "Capacity flow rate"
      annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
    Buildings.Controls.OBC.CDL.Continuous.Product loaHHW "Heating load"
      annotation (Placement(transformation(extent={{0,74},{20,94}})));
    Buildings.Controls.OBC.CDL.Continuous.Add heaFloEvaHHW
      "Heat flow rate at evaporator"
      annotation (Placement(transformation(extent={{62,-34},{82,-14}})));
    Controls.PrimaryVariableFlow conFloCon(Q_flow_nominal=Q1_flow_nominal,
        dT_nominal=dT1_nominal,
      ratFloMin=0.3)
      annotation (Placement(transformation(extent={{60,74},{80,94}})));
    Controls.PrimaryVariableFlow conFloEva(Q_flow_nominal=-Q1_flow_nominal*(1 + 1
          /COP_nominal), dT_nominal=dT2_nominal,
      ratFloMin=0.3)
      annotation (Placement(transformation(extent={{110,-34},{130,-14}})));
    Modelica.Blocks.Interfaces.RealInput yHeaPum if biomimeticControl
      "Control signal" annotation (
        Placement(transformation(extent={{-180,60},{-140,100}}),
          iconTransformation(extent={{-120,50},{-100,70}})));
    Buildings.Fluid.Sources.Boundary_pT pRef(redeclare package Medium = Medium1,
        nPorts=1) "Reference pressure"
      annotation (Placement(transformation(extent={{-20,-30},{-40,-10}})));
    Modelica.Blocks.Sources.Constant TSetSta(
      k=conHeaPum.T0) if not biomimeticControl
      "Static setpoint temperature if not biomimetic control"
      annotation (Placement(transformation(extent={{-130,-80},{-110,-60}})));
  equation
    connect(port_a1, port_a1)
      annotation (Line(points={{-140,20},{-140,20}}, color={0,127,255}));
    connect(staPum[1].y,ena. u1)
      annotation (Line(points={{-42,-80},{-60,-80}},     color={255,0,255}));
    connect(staPum[2].y,ena. u2) annotation (Line(points={{-42,-80},{-52,-80},{-52,
            -88},{-60,-88}},          color={255,0,255}));
    connect(port_a2, pumEva.port_a)
      annotation (Line(points={{140,-100},{120,-100}},
                                                     color={0,127,255}));
    connect(port_b1, senTConLea.port_b)
      annotation (Line(points={{140,20},{82,20}},  color={0,127,255}));
    connect(senTConLea.port_a, heaPum.port_b1) annotation (Line(points={{62,20},{40,
            20},{40,-24},{30,-24}},  color={0,127,255}));
    connect(port_a1, senMasFloHeaWat.port_a)
      annotation (Line(points={{-140,20},{-120,20}}, color={0,127,255}));
    connect(senMasFloHeaWat.port_b, senTConEnt.port_a)
      annotation (Line(points={{-100,20},{-80,20}}, color={0,127,255}));
    connect(pumCon.port_b, heaPum.port_a1) annotation (Line(points={{-20,20},{0,20},
            {0,-24},{10,-24}},color={0,127,255}));
    connect(senTConLea.T, dTHHW.u2) annotation (Line(points={{72,31},{72,50},{-48,
            50},{-48,64},{-42,64}}, color={0,0,127}));
    connect(senTConEnt.T, dTHHW.u1)
      annotation (Line(points={{-70,31},{-70,76},{-42,76}}, color={0,0,127}));
    connect(capFloHHW.u, senMasFloHeaWat.m_flow) annotation (Line(points={{-102,90},
            {-110,90},{-110,31}},  color={0,0,127}));
    connect(senTConEnt.port_b, pumCon.port_a)
      annotation (Line(points={{-60,20},{-40,20}}, color={0,127,255}));
    connect(capFloHHW.y, loaHHW.u1)
      annotation (Line(points={{-78,90},{-2,90}},   color={0,0,127}));
    connect(loaHHW.u2, dTHHW.y) annotation (Line(points={{-2,78},{-14,78},{-14,70},
            {-18,70}}, color={0,0,127}));
    connect(heaPum.P, heaFloEvaHHW.u2)
      annotation (Line(points={{31,-30},{60,-30}}, color={0,0,127}));
    connect(loaHHW.y, heaFloEvaHHW.u1) annotation (Line(points={{22,84},{50,84},{50,
            -18},{60,-18}}, color={0,0,127}));
    connect(loaHHW.y, conFloCon.loa)
      annotation (Line(points={{22,84},{58,84}}, color={0,0,127}));
    connect(u, conFloCon.u) annotation (Line(points={{-160,120},{50,120},{50,92},{
            58,92}}, color={255,0,255}));
    connect(heaFloEvaHHW.y, conFloEva.loa)
      annotation (Line(points={{84,-24},{108,-24}}, color={0,0,127}));
    connect(u, conFloEva.u) annotation (Line(points={{-160,120},{90,120},{90,-16},
            {108,-16}}, color={255,0,255}));
    connect(conFloEva.m_flow, pumEva.m_flow_in) annotation (Line(points={{132,-24},
            {136,-24},{136,-70},{110,-70},{110,-88}}, color={0,0,127}));
    connect(conFloCon.m_flow, pumCon.m_flow_in) annotation (Line(points={{82,84},{
            86,84},{86,40},{-30,40},{-30,32}}, color={0,0,127}));
    connect(pumCon.m_flow_actual, staPum[1].u) annotation (Line(points={{-19,25},{
            -10,25},{-10,-80},{-18,-80}}, color={0,0,127}));
    connect(pumEva.m_flow_actual, staPum[2].u) annotation (Line(points={{99,-95},{
            99,-94},{-10,-94},{-10,-80},{-18,-80}}, color={0,0,127}));
    connect(ena.y, conHeaPum.uEna) annotation (Line(points={{-84,-80},{-90,-80},{-90,
            -38},{-82,-38}},     color={255,0,255}));
    connect(yHeaPum, conHeaPum.y) annotation (Line(points={{-160,80},{-130,80},{-130,
            -44},{-82,-44}}, color={0,0,127}));
    connect(senTConEnt.T, conHeaPum.TConEnt) annotation (Line(points={{-70,31},{-70,
            40},{-90,40},{-90,-32},{-82,-32}}, color={0,0,127}));
    connect(pumEva.P, addPum.u2) annotation (Line(points={{99,-91},{98,-91},{98,64},
            {108,64}}, color={0,0,127}));
    connect(pumCon.P, addPum.u1) annotation (Line(points={{-19,29},{94,29},{94,76},
            {108,76}}, color={0,0,127}));
    connect(conHeaPum.TSet, heaPum.TSet) annotation (Line(points={{-59,-40},{-6,-40},
            {-6,-22},{8,-22},{8,-21}}, color={0,0,127}));
    connect(heaPum.P, PHea) annotation (Line(points={{31,-30},{36,-30},{36,110},{150,
            110}}, color={0,0,127}));
    connect(addPum.y, PPum)
      annotation (Line(points={{132,70},{150,70}}, color={0,0,127}));
    connect(pRef.ports[1], pumCon.port_a) annotation (Line(points={{-40,-20},{-48,
            -20},{-48,20},{-40,20}}, color={0,127,255}));
    connect(port_b2, heaPum.port_b2) annotation (Line(points={{-140,-100},{0,-100},
            {0,-36},{10,-36}},color={0,127,255}));
    connect(heaPum.port_a2, pumEva.port_b) annotation (Line(points={{30,-36},{40,
            -36},{40,-100},{100,-100}}, color={0,127,255}));
    connect(TSetSta.y, conHeaPum.TSetSta) annotation (Line(points={{-109,-70},{-100,
            -70},{-100,-50},{-82,-50}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
          Rectangle(
            extent={{-100,14},{100,26}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-100,-86},{100,-74}},
            lineColor={0,0,0},
            pattern=LinePattern.None,
            fillColor={0,0,0},
            fillPattern=FillPattern.Solid),
          Rectangle(
            extent={{-72,40},{72,-100}},
            lineColor={27,0,55},
            fillColor={170,213,255},
            fillPattern=FillPattern.Solid)}),                      Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}})));
  end HeatPump;

  package Controls "Controls for equipment subsystems"
    model HeatPump "Heat pump control"
      extends Modelica.Blocks.Icons.Block;
      parameter Boolean biomimeticControl = true
        "True if biomimetic control is enabled. False for standard control practice.";
      parameter Modelica.SIunits.Temperature TMin=273.15+15 "Minimimum desired threshold for independent variable";
      parameter Modelica.SIunits.Temperature TMax=273.15+25 "Maximum desired threshold for independent variable";
      parameter Modelica.SIunits.Temperature T0=273.15+20 "Nominal value for independent variable";
      parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
      parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
      parameter Modelica.SIunits.Time riseTime=1
        "Rise time of the filter (time to reach 99.6 % of the transition speed)";
      parameter Modelica.SIunits.Temperature THeaWatSup_nominal=313.15
        "Heating water supply temperature"
        annotation (Dialog(group="Nominal condition"));
      Modelica.Blocks.Interfaces.RealOutput TSet(
        final quantity="ThermodynamicTemperature",
        final unit="K") "Setpoint temperature"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Modelica.Blocks.Interfaces.RealInput TConEnt(
        final quantity="ThermodynamicTemperature",
        final unit="K") "Setpoint temperatureMeasured entering condenser water temperature"
        annotation (Placement(transformation(extent={{-140,-100},{-100,-60}})));
      Buildings.Controls.OBC.CDL.Logical.Switch enaHeaPum(u2(start=false))
        "Enable heat pump by switching to actual set point"
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
      Modelica.Blocks.Interfaces.BooleanInput uEna "Enable signal"
        annotation (Placement(transformation(extent={{-140,-40},{-100,0}})));
      Utilities.Math.CubicHermiteInverse spl(
        final xMin=TMin,
        final xMax=TMax,
        final x0=T0) if biomimeticControl
        annotation (Placement(transformation(extent={{-80,30},{-60,50}})));
      Buildings.Controls.OBC.CDL.Continuous.Limiter lim(
        final uMax=TMax,
        final uMin=TMin) if biomimeticControl
        annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
      Modelica.Blocks.Interfaces.RealInput y if biomimeticControl
        "Control signal"
        annotation (Placement(transformation(extent={{-140,20},{-100,60}}),
            iconTransformation(extent={{-140,20},{-100,60}})));
      Modelica.Blocks.Continuous.Filter fil(
        analogFilter=Modelica.Blocks.Types.AnalogFilter.CriticalDamping,
        filterType=Modelica.Blocks.Types.FilterType.LowPass,
        order=2,
        f_cut=5/(2*Modelica.Constants.pi*riseTime),
        init=Modelica.Blocks.Types.Init.InitialOutput,
        y_start=THeaWatSup_nominal)
        "Second order filter to approximate battery transitions between charge/off/discharge/off/charge"
        annotation (Placement(transformation(extent={{72,-10},{92,10}})));
      Modelica.Blocks.Interfaces.RealInput TSetSta(
        final quantity="ThermodynamicTemperature",
        final unit="K") if not biomimeticControl
        "Static temperature setpoint if normal control"
        annotation (Placement(transformation(extent={{-140,80},{-100,120}}),
            iconTransformation(extent={{-140,80},{-100,120}})));
    equation
      connect(spl.x,lim. u)
        annotation (Line(points={{-59,40},{-42,40}},
                                                 color={0,0,127}));
      connect(uEna, enaHeaPum.u2)
        annotation (Line(points={{-120,-20},{-42,-20},{-42,0},{38,0}},
                                                   color={255,0,255}));
      connect(TConEnt, enaHeaPum.u3) annotation (Line(points={{-120,-80},{0,-80},{0,
              -8},{38,-8}}, color={0,0,127}));
      connect(lim.y, enaHeaPum.u1) annotation (Line(points={{-18,40},{0,40},{0,8},{38,
              8}},                    color={0,0,127}));
      connect(y, spl.y)
        annotation (Line(points={{-120,40},{-102,40},{-102,40},{-82,40}},
                                                      color={0,0,127}));
      connect(enaHeaPum.y, fil.u)
        annotation (Line(points={{62,0},{70,0}}, color={0,0,127}));
      connect(fil.y, TSet)
        annotation (Line(points={{93,0},{98,0},{98,0},{110,0}}, color={0,0,127}));
      connect(TSetSta, enaHeaPum.u1) annotation (Line(points={{-120,100},{20,100},{
              20,8},{38,8}},
                          color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end HeatPump;

    block PrimaryVariableFlow
      "Ideal control of condenser or evaporator variable flow rate"
      extends Modelica.Blocks.Icons.Block;
      parameter Modelica.SIunits.HeatFlowRate Q_flow_nominal
        "Heat flow rate at nominal conditions (>0 for condenser)";
      parameter Modelica.SIunits.TemperatureDifference dT_nominal(
        min=if Q_flow_nominal>0 then Modelica.Constants.eps else -100,
        max=if Q_flow_nominal<0 then -Modelica.Constants.eps else 100)
        "DeltaT at nominal conditions (>0 for condenser)";
      parameter Real ratFloMin(
        final unit="1",
        final min=0,
        final max=1)=0.3
        "Minimum mass flow rate (ratio to nominal)";
      constant Modelica.SIunits.SpecificHeatCapacity cp=
        Buildings.Utilities.Psychrometrics.Constants.cpWatLiq
        "Specific heat capacity";
      final parameter Modelica.SIunits.MassFlowRate m_flow_nominal(min=0)=
        Q_flow_nominal/cp/dT_nominal
        "Mass flow rate at nominal conditions";
      Buildings.Controls.OBC.CDL.Interfaces.RealOutput m_flow(final unit="kg/s")
        "Mass flow rate"
        annotation (Placement(transformation(extent={{100,-20},{140,20}}),
          iconTransformation(extent={{100,-20},{140,20}})));
      Buildings.Controls.OBC.CDL.Continuous.Sources.Constant masFloMin(
        final k=ratFloMin*m_flow_nominal)
        "Minimum mass flow rate"
        annotation (Placement(transformation(extent={{-30,30},{-10,50}})));
      Buildings.Controls.OBC.CDL.Continuous.Gain masFlo_dT(
        final k=1/cp/dT_nominal)
        "Mass flow rate for constant DeltaT"
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
      Buildings.Controls.OBC.CDL.Continuous.Max masFlo "Mass flow rate"
        annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Buildings.Controls.OBC.CDL.Continuous.Abs abs1 "Absolute value"
        annotation (Placement(transformation(extent={{-30,-10},{-10,10}})));
      Buildings.Controls.OBC.CDL.Conversions.BooleanToReal booToRea
        annotation (Placement(transformation(extent={{-80,70},{-60,90}})));
      Buildings.Controls.OBC.CDL.Continuous.Product flo
        "Zero flow rate if not enabled"
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Modelica.Blocks.Interfaces.BooleanInput u "Enable signal"
        annotation (Placement(transformation(extent={{-140,60},{-100,100}})));
      Modelica.Blocks.Interfaces.RealInput loa "Load"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
    equation
      connect(masFloMin.y, masFlo.u1) annotation (Line(points={{-8,40},{0,40},{0,6},
              {18,6}},    color={0,0,127}));
      connect(masFlo_dT.y, abs1.u)
        annotation (Line(points={{-58,0},{-32,0}}, color={0,0,127}));
      connect(abs1.y, masFlo.u2)
        annotation (Line(points={{-8,0},{0,0},{0,-6},{18,-6}},   color={0,0,127}));
      connect(masFlo.y, flo.u2)
        annotation (Line(points={{42,0},{50,0},{50,-6},{58,-6}}, color={0,0,127}));
      connect(booToRea.y, flo.u1) annotation (Line(points={{-58,80},{50,80},{50,6},{
              58,6}}, color={0,0,127}));
      connect(flo.y, m_flow)
        annotation (Line(points={{82,0},{120,0}}, color={0,0,127}));
      connect(booToRea.u, u)
        annotation (Line(points={{-82,80},{-120,80}}, color={255,0,255}));
      connect(masFlo_dT.u, loa)
        annotation (Line(points={{-82,0},{-120,0}}, color={0,0,127}));
      annotation (
        defaultComponentName="conFlo",
        Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        Documentation(info="<html>
<p>
This block implements an ideal control of the evaporator (or condenser) water
mass flow rate.
The control intent aims to maintain a constant water temperature difference
<code>dT_nominal</code> across the heat exchanger, within the limit of a
minimum mass flow rate ratio <code>ratFloMin</code>.
For computational performance and to avoid the use of a PI controller,
the required mass flow rate is computed based on a signal representative of
the load.
</p>
</html>",     revisions="<html>
<ul>
<li>
February 23, 2021, by Antoine Gautier:<br/>
First implementation.
</li>
</ul>
</html>"));
    end PrimaryVariableFlow;

    model Pump "Pump control"
      extends Modelica.Blocks.Icons.Block;
      parameter Real TMin=273.15+15 "Minimimum desired threshold for independent variable";
      parameter Real TMax=273.15+25 "Maximum desired threshold for independent variable";
      parameter Real T0=273.15+20 "Nominal value for independent variable";
      parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
      parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
      Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Utilities.Math.CubicHermiteInverse spl(
        final xMin=TMin,
        final xMax=TMax,
        final x0=T0)
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
      Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(final uMax=TMax, final
          uMin=TMin)
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Modelica.Blocks.Interfaces.RealInput TMea "Measured temperature"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Modelica.Blocks.Interfaces.RealInput y "Control signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Buildings.Controls.Continuous.LimPID conPID(k=0.1, Ti=60,
        yMin=1e-3)
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      connect(spl.x, TSet.u)
        annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
      connect(y, spl.y)
        annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
      connect(TSet.y, conPID.u_s)
        annotation (Line(points={{-18,0},{38,0}}, color={0,0,127}));
      connect(TMea, conPID.u_m)
        annotation (Line(points={{-120,-60},{50,-60},{50,-12}}, color={0,0,127}));
      connect(conPID.y, yOut)
        annotation (Line(points={{61,0},{80,0},{80,0},{110,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Pump;

    model Pump2 "Pump control"
      extends Modelica.Blocks.Icons.Block;
      parameter Boolean biomimeticControl=true
        "True if biomimetic control is enabled. False for standard control practice.";
      parameter Modelica.SIunits.Temperature TMin=288.15
        "Minimimum desired threshold for independent variable";
      parameter Modelica.SIunits.Temperature TMax=298.15
        "Maximum desired threshold for independent variable";
      parameter Modelica.SIunits.Temperature T0=293.15
        "Nominal value for independent variable";
      parameter Modelica.SIunits.TemperatureDifference dT=0.5
        "Temperature deadband for complete linear transition";
      parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
      parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
      Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Utilities.Math.CubicHermiteInverse spl(
        final xMin=TMin,
        final xMax=TMax,
        final x0=T0) if biomimeticControl
        "Spline to inversely calculate pulsing setpoint from control signal"
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
      Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(
        final uMax=TMax,
        final uMin=TMin) if biomimeticControl
        "Temperature setpoint if biomimetic control"
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Modelica.Blocks.Interfaces.RealInput TSetSta if not biomimeticControl
        "Static temperature setpoint"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Modelica.Blocks.Interfaces.RealInput TMea
        "Measured temperature"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Modelica.Blocks.Interfaces.RealInput y if biomimeticControl
        "Control signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));
      Buildings.Controls.OBC.CDL.Continuous.Line lin
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
      Modelica.Blocks.Sources.Constant off(k=0.01) "Off signal"
        annotation (Placement(transformation(extent={{0,-50},{20,-30}})));
      Modelica.Blocks.Sources.Constant on(k=1) "On signal"
        annotation (Placement(transformation(extent={{0,50},{20,70}})));
      Modelica.Blocks.Sources.Constant dTSet(k=dT)   "Transition zone"
        annotation (Placement(transformation(extent={{-40,30},{-20,50}})));
      Modelica.Blocks.Math.Add x1(k1=-1) "First transition point"
        annotation (Placement(transformation(extent={{0,10},{20,30}})));

    equation
      connect(spl.x, TSet.u)
        annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
      connect(y, spl.y)
        annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
      connect(TMea, lin.u) annotation (Line(points={{-120,-60},{50,-60},{50,0},{58,0}},
            color={0,0,127}));
      connect(off.y, lin.f2) annotation (Line(points={{21,-40},{40,-40},{40,-8},{58,
              -8}},
            color={0,0,127}));
      connect(TSet.y, x1.u2) annotation (Line(points={{-18,0},{-10,0},{-10,14},{-2,14}},
                    color={0,0,127}));
      connect(dTSet.y, x1.u1) annotation (Line(points={{-19,40},{-14,40},{-14,26},{-2,
              26}}, color={0,0,127}));
      connect(x1.y, lin.x1)
        annotation (Line(points={{21,20},{30,20},{30,8},{58,8}},color={0,0,127}));
      connect(on.y, lin.f1)
        annotation (Line(points={{21,60},{40,60},{40,4},{58,4}},color={0,0,127}));
      connect(TSet.y, lin.x2) annotation (Line(points={{-18,0},{-10,0},{-10,-4},{58,
              -4}},
            color={0,0,127}));
      connect(lin.y, yOut)
        annotation (Line(points={{82,0},{110,0}}, color={0,0,127}));
      connect(TSetSta, x1.u2) annotation (Line(points={{-120,60},{-70,60},{-70,20},{
              -10,20},{-10,14},{-2,14}}, color={0,0,127}));
      connect(TSetSta, lin.x2) annotation (Line(points={{-120,60},{-70,60},{-70,20},
              {-10,20},{-10,-4},{58,-4}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Pump2;

    model Pump3 "Pump control - on/off"
      extends Modelica.Blocks.Icons.Block;
      parameter Boolean biomimeticControl=true
        "True if biomimetic control is enabled. False for standard control practice.";
      parameter Modelica.SIunits.Temperature TMin=288.15
        "Minimimum desired threshold for independent variable";
      parameter Modelica.SIunits.Temperature TMax=298.15
        "Maximum desired threshold for independent variable";
      parameter Modelica.SIunits.Temperature T0=293.15
        "Nominal value for independent variable";
      parameter Modelica.SIunits.TemperatureDifference dT=1
        "Temperature deadband for complete linear transition";
      parameter Real a(min=0,max=1) = 0.5 "First weighting factor";
      parameter Real b(min=0,max=1) = 1 - a "First weighting factor";
      Modelica.Blocks.Interfaces.RealOutput yOut "Speed setpoint for pump/fan"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Utilities.Math.CubicHermiteInverse spl(
        final xMin=TMin,
        final xMax=TMax,
        final x0=T0) if biomimeticControl
        "Spline to inversely calculate pulsing setpoint from control signal"
        annotation (Placement(transformation(extent={{-80,-10},{-60,10}})));
      Buildings.Controls.OBC.CDL.Continuous.Limiter TSet(
        final uMax=TMax,
        final uMin=TMin) if biomimeticControl
        "Temperature setpoint if biomimetic control"
        annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
      Modelica.Blocks.Interfaces.RealInput TSetSta if not biomimeticControl
        "Static temperature setpoint"
        annotation (Placement(transformation(extent={{-140,40},{-100,80}})));
      Modelica.Blocks.Interfaces.RealInput TMea
        "Measured temperature"
        annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
      Modelica.Blocks.Interfaces.RealInput y if biomimeticControl
        "Control signal"
        annotation (Placement(transformation(extent={{-140,-20},{-100,20}})));

      Buildings.Controls.OBC.CDL.Logical.OnOffController onOffCon(bandwidth=dT)
        annotation (Placement(transformation(extent={{20,-10},{40,10}})));
      Modelica.Blocks.Math.BooleanToReal booToRea(realFalse=0.02)
        "Convert boolean to real"
        annotation (Placement(transformation(extent={{60,-10},{80,10}})));
    equation
      connect(spl.x, TSet.u)
        annotation (Line(points={{-59,0},{-42,0}}, color={0,0,127}));
      connect(y, spl.y)
        annotation (Line(points={{-120,0},{-82,0}}, color={0,0,127}));
      connect(TMea, onOffCon.u) annotation (Line(points={{-120,-60},{0,-60},{0,-6},
              {18,-6}}, color={0,0,127}));
      connect(TSet.y, onOffCon.reference)
        annotation (Line(points={{-18,0},{0,0},{0,6},{18,6}}, color={0,0,127}));
      connect(TSetSta, onOffCon.reference)
        annotation (Line(points={{-120,60},{0,60},{0,6},{18,6}}, color={0,0,127}));
      connect(onOffCon.y, booToRea.u)
        annotation (Line(points={{42,0},{58,0}}, color={255,0,255}));
      connect(booToRea.y, yOut)
        annotation (Line(points={{81,0},{92,0},{92,0},{110,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end Pump3;
  end Controls;
end Device;
