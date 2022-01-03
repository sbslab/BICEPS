within BICEPS.Combined.ElectricalAndFluid.Examples;
model BiomimeticControl
  "SingleFamilyResidentialBuilding with biomimetic control"
  extends SingleFamilyResidentialBuilding(bld(biomimeticControl=true));
  annotation (__Dymola_Commands(
    file="modelica://BICEPS/Resources/Scripts/Dymola/Combined/ElectricalAndFluid/Examples//NormalAndBiomimeticControl.mos"
        "Normal and Biomimetic"));
end BiomimeticControl;