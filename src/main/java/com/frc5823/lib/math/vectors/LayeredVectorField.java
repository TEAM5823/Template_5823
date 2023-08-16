package com.frc5823.lib.math.vectors;

import com.frc5823.lib.geometry.Translation2d;

import java.util.ArrayList;
import java.util.List;

public class LayeredVectorField extends VectorField {
  public LayeredVectorField(List<VectorField> fields_) {
    for (VectorField f : fields_) fields.add(f);
  }

  public List<VectorField> fields = new ArrayList<VectorField>(); // public, so may add() and such

  public Translation2d getVector(Translation2d here) {
    Translation2d out = new Translation2d(0.0, 0.0);
    for (VectorField f : fields) out = out.translateBy(f.getVector(here));
    return out.normalize();
  }
}
