within BICEPS.Fluid;
package Building "Thermofluid models at the building level"
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
    Controls.ThermoFluid conFlu(n=2,
      a={0.9,0.1},                   tSmo=tSmo) if biomimeticControl
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

  model ThermoFluidOneElement "Thermofluid subsystem"
    replaceable package MediumWat=Buildings.Media.Water
      constrainedby Modelica.Media.Interfaces.PartialMedium
      "Medium in the building distribution system";
    replaceable package MediumAir=Buildings.Media.Air
      constrainedby Modelica.Media.Interfaces.PartialMedium
      "Load side medium";
    parameter Modelica.SIunits.HeatFlowRate QHea_flow_nominal=0
      "Nominal heating capacity (>=0)";
    parameter Real COP_nominal "Heat pump COP";
    parameter Modelica.SIunits.Pressure dp_nominal(displayUnit="Pa") = 50000
      "Pressure difference at nominal flow rate (for each flow leg)"
      annotation(Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.TemperatureDifference dT_nominal(min=0) = 5
      "Water temperature drop/increase accross load and source-side HX (always positive)"
      annotation (Dialog(group="Nominal condition"));
    parameter Modelica.SIunits.Temperature TDisWatMin=6+273.15
      "District water minimum temperature"
      annotation (Dialog(group="DHC system"));
    parameter Modelica.SIunits.Temperature THeaWatSup_nominal=313.15
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
    Device.HeatPump heaPum(
      redeclare package Medium1 = MediumWat,
      redeclare package Medium2 = MediumWat,
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
      redeclare package Medium1 = MediumWat,
      redeclare package Medium2 = MediumAir,
      m1_flow_nominal=mHeaWat_flow_nominal,
      m2_flow_nominal=mLoaHea_flow_nominal,
      dp1_nominal=100000,
      dp2_nominal=250,
      QHea_flow_nominal=QHea_flow_nominal)
      annotation (Placement(transformation(extent={{-30,-20},{-10,0}})));
    ThermalZones.SimpleRoomOneElement zon
      annotation (Placement(transformation(extent={{-30,20},{-10,40}})));
    Modelica.Blocks.Interfaces.RealInput yEle
      "Relative exergetic potential of electrical subsystem"
      annotation (Placement(transformation(extent={{-140,50},{-100,90}})));
    Modelica.Blocks.Interfaces.RealOutput PHeaPum(
      final quantity="Power",
      final unit="W",
      min=0,
      displayUnit="kW")   "Heat pump power"
      annotation (Placement(transformation(extent={{100,40},{120,60}})));
    Buildings.Controls.OBC.CDL.Interfaces.RealOutput PPum(final unit="W")
      "Pump power"
      annotation (Placement(transformation(extent={{100,70},{120,90}}),
      iconTransformation(extent={{100,70},{140,110}})));
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
    Controls.ThermoFluid conFlu(n=2)
      annotation (Placement(transformation(extent={{40,20},{60,40}})));

    Modelica.Blocks.Logical.Hysteresis enaHea(uLow=1, uHigh=1.01)
      "Enable heating if less than 1 (TRoom < TMax)"
      annotation (Placement(transformation(extent={{60,-20},{40,0}})));
    Modelica.Blocks.Logical.Not not1 "Invert"
      annotation (Placement(transformation(extent={{30,-20},{10,0}})));
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
        points={{0,100},{0,88},{-20,88},{-20,40}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{-3,6},{-3,6}},
        horizontalAlignment=TextAlignment.Right));
    connect(heaPum.PHea, PHeaPum) annotation (Line(points={{-32,-42},{-60,-42},{
            -60,50},{110,50}},
                           color={0,0,127}));
    connect(heaPum.PPum, PPum) annotation (Line(points={{-32,-46},{-64,-46},{-64,
            80},{110,80}},
                       color={0,0,127}));
    connect(yEle, conFlu.yIn[1]) annotation (Line(points={{-120,70},{34,70},{34,32},
            {40,32},{40,29}}, color={0,0,127}));
    connect(zon.y, conFlu.yIn[2]) annotation (Line(points={{-9,37},{20,37},{20,30},
            {40,30},{40,31}}, color={0,0,127}));
    connect(conFlu.yOut, heaPum.yHeaPum) annotation (Line(points={{61,30},{80,30},
            {80,-46},{-9,-46}}, color={0,0,127}));
    connect(zon.y, enaHea.u) annotation (Line(points={{-9,37},{20,37},{20,10},{70,
            10},{70,-10},{62,-10}}, color={0,0,127}));
    connect(enaHea.y, not1.u)
      annotation (Line(points={{39,-10},{32,-10}}, color={255,0,255}));
    connect(not1.y, heaPum.u) annotation (Line(points={{9,-10},{4,-10},{4,-42},{
            -9,-42}},
                   color={255,0,255}));
    connect(conFlu.yOut, fcu.y) annotation (Line(points={{61,30},{80,30},{80,14},
            {-36,14},{-36,-2},{-32,-2}}, color={0,0,127}));
    annotation (Icon(coordinateSystem(preserveAspectRatio=false), graphics={
            Rectangle(
            extent={{-80,80},{80,-80}},
            lineColor={0,0,0},
            fillColor={28,108,200},
            fillPattern=FillPattern.Solid),
          Rectangle(extent={{-40,22},{40,-58}}, lineColor={0,0,0}),
          Line(points={{-40,22},{0,62},{40,22}}, color={0,0,0})}),
        Diagram(coordinateSystem(preserveAspectRatio=false)));
  end ThermoFluidOneElement;

  package Controls "Package with controls for fluid subsystems"
    model ThermoFluid "Thermofluid subsystem control block"
      extends Modelica.Blocks.Icons.Block;
      parameter Integer n(min=1) "Number of input connectors";
      parameter Real a[n] = fill(1/n, n) "Weighting factors";
      parameter Real tSmo(
        final quantity="Time",
        final unit="s",
        min=1E-5)=30*60
        "Smoothing time for thermal-fluid control signal";
      Modelica.Blocks.Math.MultiSum multiSum(each k=a, nu=n)
        annotation (Placement(transformation(extent={{-56,-6},{-44,6}})));
      Modelica.Blocks.Math.Gain nor(k=sum(a)) "Normalized signal"
        annotation (Placement(transformation(extent={{0,-10},{20,10}})));
      Modelica.Blocks.Interfaces.RealVectorInput yIn[n] "Input control signals"
        annotation (Placement(transformation(extent={{-120,-20},{-80,20}})));
      Modelica.Blocks.Interfaces.RealOutput yOut "Output control signal"
        annotation (Placement(transformation(extent={{100,-10},{120,10}})));
      Buildings.Controls.OBC.CDL.Continuous.MovingMean smo(delta=tSmo)
        "Smoothing via a moving mean to account for thermal inertia"
        annotation (Placement(transformation(extent={{40,-10},{60,10}})));
    equation
      connect(multiSum.y,nor. u)
        annotation (Line(points={{-42.98,0},{-2,0}}, color={0,0,127}));
      connect(yIn,multiSum. u)
        annotation (Line(points={{-100,0},{-56,0}},color={0,0,127}));
      connect(nor.y, smo.u)
        annotation (Line(points={{21,0},{38,0}}, color={0,0,127}));
      connect(smo.y, yOut)
        annotation (Line(points={{62,0},{110,0}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ThermoFluid;
  end Controls;

  package Examples "Collection of models that illustrate model use and test models"
    extends Modelica.Icons.ExamplesPackage;

    model ThermoFluid1E "Test model for the thermofluid subsystem"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium in the building distribution system";
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                                weaDat(
        calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
        computeWetBulbTemperature=false,
        filNam=Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
        "Weather data reader"
        annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

      BICEPS.Fluid.Building.ThermoFluidOneElement thermoFluid(
        redeclare package MediumWat = Medium,
        QHea_flow_nominal=261700,
        COP_nominal=4,
        mLoaHea_flow_nominal=10)
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Modelica.Blocks.Sources.Constant idealElecSig(k=0) "Ideal electrical signal"
        annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
      Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium,
        T=thermoFluid.TDisWatMin,
        nPorts=1) "Source"
        annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
      Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = Medium,
          nPorts=1) "Sink"
        annotation (Placement(transformation(extent={{62,0},{42,20}})));
    equation
      connect(weaDat.weaBus, thermoFluid.weaBus) annotation (Line(
          points={{-20,70},{10,70},{10,40}},
          color={255,204,51},
          thickness=0.5));
      connect(idealElecSig.y, thermoFluid.yEle) annotation (Line(points={{-59,70},{-50,
              70},{-50,37},{-2,37}}, color={0,0,127}));
      connect(sou.ports[1], thermoFluid.port_a) annotation (Line(points={{-20,10},{-10,
              10},{-10,24},{0,24}}, color={0,127,255}));
      connect(thermoFluid.port_b, sin.ports[1]) annotation (Line(points={{20,24},{30,
              24},{30,10},{42,10}}, color={0,127,255}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
        __Dymola_Commands(
          file="modelica://BICEPS/Resources/Scripts/Dymola/Fluid/Building/Examples/ThermoFluid1E.mos"
          "Simulate and plot"),
        experiment(
          StopTime=86400,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),
        Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ThermoFluid1E;

    model ThermoFluid1ESimpleDistrictLoop
      "Test model for the thermofluid subsystem"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium in the building distribution system";
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                                weaDat(
        calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
        computeWetBulbTemperature=false,
        filNam=Modelica.Utilities.Files.loadResource("modelica://Buildings/Resources/weatherdata/USA_IL_Chicago-OHare.Intl.AP.725300_TMY3.mos"))
        "Weather data reader"
        annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

      BICEPS.Fluid.Building.ThermoFluidOneElement thermoFluid(
        redeclare package MediumWat = Medium,
        QHea_flow_nominal=261700,
        COP_nominal=4,
        mLoaHea_flow_nominal=10)
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Modelica.Blocks.Sources.Constant ideal(k=0)
        annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
      Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium,
          nPorts=1) "Source"
        annotation (Placement(transformation(extent={{80,-40},{60,-20}})));
      Buildings.Experimental.DHC.EnergyTransferStations.BaseClasses.Pump_m_flow
                                                         pumDis(
        redeclare package Medium = Medium,
        final m_flow_nominal=95,
        inputType=Buildings.Fluid.Types.InputType.Constant,
        addPowerToMedium=false,
        nominalValuesDefineDefaultPressureCurve=true)
        "Distribution pump"
        annotation (Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=90,
          origin={50,-50})));
      Buildings.Fluid.HeatExchangers.Heater_T pla(
        redeclare package Medium = Medium,
        m_flow_nominal=thermoFluid.mHeaWat_flow_nominal,
        dp_nominal=0)                             "Ideal plant"
        annotation (Placement(transformation(extent={{20,-80},{0,-60}})));
      Buildings.Fluid.FixedResistances.Junction spl(
        redeclare package Medium = Medium,
        m_flow_nominal=thermoFluid.mHeaWat_flow_nominal*{1,-1,-1},
        dp_nominal={0,0,0}) "Splitter"
        annotation (Placement(transformation(extent={{-20,-10},{0,-30}})));
      Buildings.Fluid.FixedResistances.Junction jun(
        redeclare package Medium = Medium,
        m_flow_nominal=thermoFluid.mHeaWat_flow_nominal*{1,1,-1},
        dp_nominal={0,0,0}) "Junction"
        annotation (Placement(transformation(extent={{20,-10},{40,-30}})));
      Buildings.Fluid.FixedResistances.PressureDrop pipDis(
        redeclare package Medium = Medium,
        m_flow_nominal=thermoFluid.mHeaWat_flow_nominal,
        dp_nominal=60000) "District pipe" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-40,-50})));
      Buildings.Fluid.FixedResistances.PressureDrop pipBld(
        redeclare package Medium = Medium,
        m_flow_nominal=thermoFluid.mHeaWat_flow_nominal,
        dp_nominal=6000) "Building pipe" annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=90,
            origin={-10,6})));
      Modelica.Blocks.Sources.Constant TSetDis(k=thermoFluid.TDisWatMin)
        "District loop temperature"
        annotation (Placement(transformation(extent={{80,-90},{60,-70}})));
    equation
      connect(weaDat.weaBus, thermoFluid.weaBus) annotation (Line(
          points={{-20,70},{10,70},{10,40}},
          color={255,204,51},
          thickness=0.5));
      connect(ideal.y, thermoFluid.yEle) annotation (Line(points={{-59,70},{-50,70},
              {-50,37},{-2,37}}, color={0,0,127}));
      connect(sou.ports[1], pumDis.port_a)
        annotation (Line(points={{60,-30},{50,-30},{50,-40}}, color={0,127,255}));
      connect(pumDis.port_b, pla.port_a)
        annotation (Line(points={{50,-60},{50,-70},{20,-70}}, color={0,127,255}));
      connect(pla.port_b, pipDis.port_a)
        annotation (Line(points={{0,-70},{-40,-70},{-40,-60}}, color={0,127,255}));
      connect(pipDis.port_b, spl.port_1) annotation (Line(points={{-40,-40},{-40,
              -20},{-20,-20}}, color={0,127,255}));
      connect(spl.port_2, jun.port_1)
        annotation (Line(points={{0,-20},{20,-20}}, color={0,127,255}));
      connect(pumDis.port_a, jun.port_2)
        annotation (Line(points={{50,-40},{50,-20},{40,-20}}, color={0,127,255}));
      connect(spl.port_3, pipBld.port_a)
        annotation (Line(points={{-10,-10},{-10,-4}}, color={0,127,255}));
      connect(pipBld.port_b, thermoFluid.port_a)
        annotation (Line(points={{-10,16},{-10,24},{0,24}},  color={0,127,255}));
      connect(thermoFluid.port_b, jun.port_3)
        annotation (Line(points={{20,24},{30,24},{30,-10}}, color={0,127,255}));
      connect(TSetDis.y, pla.TSet) annotation (Line(points={{59,-80},{30,-80},{30,
              -62},{22,-62}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        experiment(StopTime=86400, Tolerance=1e-6, __Dymola_Algorithm="Dassl"));
    end ThermoFluid1ESimpleDistrictLoop;

    model ThermoFluid4E "Test model for the thermofluid subsystem"
      extends Modelica.Icons.Example;
      package Medium=Buildings.Media.Water
        "Medium in the building distribution system";
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3
                                                weaDat(
        calTSky=Buildings.BoundaryConditions.Types.SkyTemperatureCalculation.HorizontalRadiation,
        computeWetBulbTemperature=false,
        filNam=ModelicaServices.ExternalReferences.loadResource(
            "modelica://BICEPS/Resources/weatherdata/DEU_BW_Mannheim_107290_TRY2010_12_Jahr_BBSR.mos"))
        "Weather data reader"
        annotation (Placement(transformation(extent={{-40,60},{-20,80}})));

      BICEPS.Fluid.Building.ThermoFluidFourElements thermoFluid(
        redeclare package MediumWat = Medium,
        biomimeticControl=true,
        show_T=true,
        QHea_flow_nominal(displayUnit="kW") = 20000,
        COP_nominal=4,
        mLoaHea_flow_nominal=1,
        TMin=thermoFluid.T0 - 1.5,
        TMax=thermoFluid.T0 + 2.5)
        annotation (Placement(transformation(extent={{0,20},{20,40}})));
      Modelica.Blocks.Sources.Constant idealElecSig(k=0) "Ideal electrical signal"
        annotation (Placement(transformation(extent={{-80,60},{-60,80}})));
      Buildings.Fluid.Sources.Boundary_pT sou(redeclare package Medium = Medium,
        T=285.15,
        nPorts=1) "Source"
        annotation (Placement(transformation(extent={{-40,0},{-20,20}})));
      Buildings.Fluid.Sources.Boundary_pT sin(redeclare package Medium = Medium,
          nPorts=1) "Sink"
        annotation (Placement(transformation(extent={{62,0},{42,20}})));
      Modelica.Blocks.Continuous.Integrator EPum "Pump energy"
        annotation (Placement(transformation(extent={{60,60},{80,80}})));
      Modelica.Blocks.Continuous.Integrator EHeaPum "Heat Pump energy"
        annotation (Placement(transformation(extent={{60,30},{80,50}})));
      Modelica.Blocks.Sources.Trapezoid yEle(
        amplitude=1,
        rising(displayUnit="h") = 3600,
        width(displayUnit="h") = 10800,
        falling(displayUnit="h") = 3600,
        period(displayUnit="h") = 32400,
        offset=-1,
        startTime(displayUnit="h") = 3600)
        annotation (Placement(transformation(extent={{-80,26},{-60,46}})));
    equation
      connect(weaDat.weaBus, thermoFluid.weaBus) annotation (Line(
          points={{-20,70},{10,70},{10,40}},
          color={255,204,51},
          thickness=0.5));
      connect(sou.ports[1], thermoFluid.port_a) annotation (Line(points={{-20,10},{-10,
              10},{-10,24},{0,24}}, color={0,127,255}));
      connect(thermoFluid.port_b, sin.ports[1]) annotation (Line(points={{20,24},{30,
              24},{30,10},{42,10}}, color={0,127,255}));
      connect(thermoFluid.PHeaPum, EHeaPum.u) annotation (Line(points={{21,35},{40,
              35},{40,40},{58,40}}, color={0,0,127}));
      connect(thermoFluid.PPum, EPum.u) annotation (Line(points={{21,38},{30,38},{
              30,70},{58,70}}, color={0,0,127}));
      connect(yEle.y, thermoFluid.yEle) annotation (Line(points={{-59,36},{-22,36},
              {-22,37},{-2,37}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)),
        __Dymola_Commands(
          file="modelica://BICEPS/Resources/Scripts/Dymola/Fluid/Building/Examples/ThermoFluid4E.mos"
          "Simulate and plot"),
        experiment(
          StopTime=86400,
          Tolerance=1e-06,
          __Dymola_Algorithm="Dassl"),
        Diagram(
            coordinateSystem(preserveAspectRatio=false)));
    end ThermoFluid4E;
  end Examples;
end Building;
