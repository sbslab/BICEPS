within BICEPS.Electrical;
package Building "Electrical models at the building level"

  model Electrical "Model of a building's electrical system"
    extends Buildings.BaseClasses.BaseIconLow;
    parameter Boolean biomimeticControl=true
      "True if biomimetic control is enabled. False for standard control practice.";
    parameter Integer nSto=1 "Number of storage connections";
    parameter Real tol=0.05
      "Tolerance allowed on nominal voltage control (5-10% typical)";
    parameter Real k=100 "Percentage penalty for deviating outside of min/max range. Smaller numbers
    indicate a steeper penalty.";
    parameter Modelica.SIunits.Angle lat "Latitude";
    parameter Modelica.SIunits.Power PCon_nominal
      "Nominal power for consumer loads";
    parameter Modelica.SIunits.Power PSun "Nominal power of the PV";
    parameter Modelica.SIunits.Power PSto_nominal
      "Nominal power for storage loads";
    parameter Modelica.SIunits.Energy EBatMax=180000000
      "Maximum energy capacity of the battery";
    parameter Modelica.SIunits.Power PBatMax(min=0)=10000
      "Maximum power charge/discharge rate";
    parameter Modelica.SIunits.Power PBatMin(min=0)=100
      "Minimum power charge/discharge rate";
    parameter Modelica.SIunits.Voltage V_nominal=208
      "Nominal voltage of the line";
    parameter Modelica.SIunits.Length LGri=1500 "Length of the grid line";
    parameter Modelica.SIunits.Length LCon=10
      "Length of the consumer lines";
    parameter Modelica.SIunits.Length LPro=25
      "Length of the producer lines";
    parameter Modelica.SIunits.Length LSto[nSto]=fill(5,nSto)
      "Length of the storage lines";
    Device.Panel P1(biomimeticControl=biomimeticControl, nSto=nSto)
      annotation (Placement(transformation(extent={{-10,20},{10,40}})));
    BaseClasses.ConnectedDevices dev(
      biomimeticControl=biomimeticControl,
      nSto=nSto,
      lat=lat,
      V_nominal=V_nominal,
      tol=tol,
      k=k,
      PSun=PSun,
      PBat=PSto_nominal,
      PBatMax=PBatMax,
      PBatMin=PBatMin,
      EBatMax=EBatMax)
      annotation (Placement(transformation(extent={{10,-40},{-10,-20}})));
    Buildings.Electrical.AC.OnePhase.Lines.Line linGri(
      l=LGri,
      P_nominal=PCon_nominal+PSun+PSto_nominal,
      final V_nominal=V_nominal) "Grid power line"
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={0,60})));
    Buildings.Electrical.AC.OnePhase.Lines.Line linCon(
      l=LCon,
      P_nominal=PCon_nominal,
      final V_nominal=V_nominal) "Consumer power lines" annotation (
        Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=-90,
          origin={10,0})));
    Buildings.Electrical.AC.OnePhase.Lines.Line linSto[nSto](
      l=LSto,
      final P_nominal=fill(PSto_nominal, nSto),
      each final V_nominal=V_nominal) "Storage  power lines" annotation (Placement(
          transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={0,0})));
    Buildings.Electrical.AC.OnePhase.Lines.Line linPro(
      l=LPro,
      P_nominal=PSun,
      final V_nominal=V_nominal) "Producer power lines" annotation (
        Placement(transformation(
          extent={{10,-10},{-10,10}},
          rotation=-90,
          origin={-10,0})));
    Buildings.Electrical.AC.ThreePhasesBalanced.Interfaces.Terminal_p terminal
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={-110,80}),    iconTransformation(extent={{-120,60},{-100,80}})));
    Modelica.Blocks.Interfaces.RealInput PCon "Total power of all consumers"
      annotation (Placement(transformation(extent={{-140,-80},{-100,-40}})));
    Buildings.BoundaryConditions.WeatherData.Bus weaBus
      "Weather data bus"
      annotation (Placement(transformation(extent={{-20,82},{20,122}}),
        iconTransformation(extent={{-10,90},{10,110}})));
    Modelica.Blocks.Interfaces.RealOutput yOut if biomimeticControl
      "Output control signal"
      annotation (Placement(transformation(extent={{100,70},{120,90}})));
  equation
    connect(P1.terCon, linCon.terminal_n) annotation (Line(points={{4,19},{4,14},{
            10,14},{10,10}}, color={0,120,120}));
    connect(P1.terSto, linSto.terminal_p)
      annotation (Line(points={{0,19},{0,10}}, color={0,120,120}));
    connect(P1.terPro, linPro.terminal_p) annotation (Line(points={{-4,19},{-4,14},
            {-10,14},{-10,10}}, color={0,120,120}));
    connect(linCon.terminal_p, dev.terCon) annotation (Line(points={{10,-10},{10,-14},
            {4,-14},{4,-19}}, color={0,120,120}));
    connect(linSto.terminal_n, dev.terSto)
      annotation (Line(points={{0,-10},{0,-19}}, color={0,120,120}));
    connect(linPro.terminal_n, dev.terPro) annotation (Line(points={{-10,-10},{-10,
            -14},{-4,-14},{-4,-19}}, color={0,120,120}));
    connect(linGri.terminal_p, P1.terGri)
      annotation (Line(points={{0,50},{0,41}}, color={0,120,120}));
    connect(terminal, linGri.terminal_n)
      annotation (Line(points={{-110,80},{0,80},{0,70}},
                                                color={0,120,120}));
    connect(PCon, dev.PCon) annotation (Line(points={{-120,-60},{30,-60},{30,-36},
            {12,-36}}, color={0,0,127}));
    connect(dev.yPro, P1.yPro) annotation (Line(points={{-11,-22},{-22,-22},{-22,30},
            {-12,30}},     color={0,0,127}));
    connect(dev.ySto, P1.ySto) annotation (Line(points={{-11,-26},{-26,-26},{-26,34},
            {-12,34}}, color={0,0,127}));
    connect(dev.yCon, P1.yCon) annotation (Line(points={{-11,-30},{-30,-30},{-30,38},
            {-12,38}}, color={0,0,127}));
    connect(weaBus, dev.weaBus) annotation (Line(
        points={{0,102},{0,88},{40,88},{40,-20},{9,-20}},
        color={255,204,51},
        thickness=0.5), Text(
        string="%first",
        index=-1,
        extent={{6,3},{6,3}},
        horizontalAlignment=TextAlignment.Left));
    connect(P1.yOut, yOut) annotation (Line(points={{11,36},{30,36},{30,80},{110,
            80}},
          color={0,0,127}));
    connect(P1.PNetOut, dev.PNetIn) annotation (Line(points={{11,32},{30,32},{30,
            -24},{12,-24}}, color={0,0,127}));
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

  package Examples "Collection of models that illustrate model use and test models"
    extends Modelica.Icons.ExamplesPackage;

    model Electrical
      "Example model to demonstrate the electrical subsystem"
      extends Modelica.Icons.Example;
      BICEPS.Electrical.Building.Electrical ele(
        biomimeticControl=false,
        lat=weaDat.lat,
        PCon_nominal=PHeaPum.k,
        PSun=5000,
        PSto_nominal=1000,
        PBatMax=2000) "Electrical subsystem"
        annotation (Placement(transformation(extent={{-20,0},{0,20}})));
      Modelica.Blocks.Continuous.Integrator EGri
        "Grid energy"
        annotation (Placement(transformation(extent={{60,60},{80,80}})));
      Modelica.Blocks.Sources.RealExpression PGriRea(y=gri.P.real)
        "Real grid power"
        annotation (Placement(transformation(extent={{20,60},{40,80}})));
      Modelica.Blocks.Continuous.Integrator Epv
        "PV energy"
        annotation (Placement(transformation(extent={{60,30},{80,50}})));
      Modelica.Blocks.Continuous.Integrator ELoa
        "Load energy"
        annotation (Placement(transformation(extent={{60,0},{80,20}})));
      Modelica.Blocks.Continuous.Integrator EBat
        "Battery energy"
        annotation (Placement(transformation(extent={{60,-30},{80,-10}})));
      Modelica.Blocks.Sources.RealExpression Ppv(y=ele.dev.pv.pv.P)
        "PV power"
        annotation (Placement(transformation(extent={{20,30},{40,50}})));
      Modelica.Blocks.Sources.RealExpression PLoa(y=ele.dev.con.loa.P)
        "Load power"
        annotation (Placement(transformation(extent={{20,0},{40,20}})));
      Modelica.Blocks.Sources.RealExpression PBat(y=-1*ele.dev.bat[1].bat.P)
        "Battery power"
        annotation (Placement(transformation(extent={{20,-30},{40,-10}})));
      Buildings.BoundaryConditions.WeatherData.ReaderTMY3 weaDat(
        computeWetBulbTemperature=false,
        filNam=Modelica.Utilities.Files.loadResource(
          "modelica://Buildings/Resources/weatherdata/USA_CA_San.Francisco.Intl.AP.724940_TMY3.mos"))
        "Weather data model"
        annotation (Placement(transformation(extent={{-40,60},{-20,80}})));
      Buildings.Electrical.AC.ThreePhasesBalanced.Sources.Grid gri(
        f=60,
        V=208,
        phiSou=0)
        "Grid model that provides power to the system"
        annotation (Placement(transformation(extent={{-80,40},{-60,60}})));
      Modelica.Blocks.Sources.Constant PHeaPum(k=500)
        "Heat pump power"
        annotation (Placement(transformation(extent={{-80,-20},{-60,0}})));
    equation
      connect(PGriRea.y,EGri. u)
        annotation (Line(points={{41,70},{58,70}}, color={0,0,127}));
      connect(Ppv.y,Epv. u)
        annotation (Line(points={{41,40},{58,40}}, color={0,0,127}));
      connect(PLoa.y,ELoa. u)
        annotation (Line(points={{41,10},{58,10}},   color={0,0,127}));
      connect(PBat.y,EBat. u)
        annotation (Line(points={{41,-20},{58,-20}}, color={0,0,127}));
      connect(gri.terminal, ele.terminal)
        annotation (Line(points={{-70,40},{-70,17},{-21,17}}, color={0,120,120}));
      connect(weaDat.weaBus, ele.weaBus) annotation (Line(
          points={{-20,70},{-10,70},{-10,20}},
          color={255,204,51},
          thickness=0.5));
      connect(PHeaPum.y, ele.PCon) annotation (Line(points={{-59,-10},{-40,-10},
              {-40,4},{-22,4}}, color={0,0,127}));
      annotation (Icon(coordinateSystem(preserveAspectRatio=false)), Diagram(
            coordinateSystem(preserveAspectRatio=false)),
        __Dymola_Commands(
          file="modelica://BICEPS/Resources/Scripts/Dymola/Electrical/Building/Examples/Electrical.mos"
          "Simulate and plot"),
        experiment(
          StartTime=8640000,
          StopTime=8726400,
          Tolerance=1e-6,
          __Dymola_Algorithm="Dassl"));
    end Electrical;
  end Examples;

  package BaseClasses "Package with base classes for electrical systems at the
  building level"
    extends Modelica.Icons.BasesPackage;
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
  end BaseClasses;
end Building;
