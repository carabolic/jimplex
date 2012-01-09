package de.tu_berlin.coga.jimplex;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.ParseException;

public class LinearProgram {
	double[][] constraints;
	double[] rightHandSide;
	double[] lowerBound;
	double[] upperBound;
	Sense[] senses;

	double[] obj;
	ObjectiveGoal objGoal;

	String[] constraintNames;
	String[] varName;
	int numVariables;

	public LinearProgram(String file) throws FileNotFoundException, ParseException, IOException {
		LPReader reader = new LPReader(file);
		reader.readLP();

		constraints = reader.constraint;
		rightHandSide = reader.rhs;
		lowerBound = reader.lbound;
		upperBound = reader.ubound;

		// Converted constraint senses
		senses = new Sense[reader.sense.length];
		for (int i = 0; i < reader.sense.length; i++) {
			switch (reader.sense[i]) {
			case LPReader.SENSE_EQ:
				senses[i] = Sense.EQ;
				break;
			case LPReader.SENSE_GEQ:
				senses[i] = Sense.GEQ;
				break;
			case LPReader.SENSE_LEQ:
				senses[i] = Sense.LEQ;
				break;
			default:
				throw new RuntimeException();
			}
		}

		obj = reader.obj;
		switch (reader.objsense) {
		case LPReader.SENSE_MAX:
			objGoal = ObjectiveGoal.MAX;
			break;
		case LPReader.SENSE_MIN:
			objGoal = ObjectiveGoal.MIN;
			break;
		default:
			throw new RuntimeException();
		}

		constraintNames = reader.constrName;
		varName = reader.varName;
		numVariables = varName.length;
	}

	/**
	 * Makes the problem a minimizing Problem Only allow equality constraints and
	 * positive (>0) variables
	 */
	public void normalize() {
		// Make all problems minimizing
		if (objGoal == ObjectiveGoal.MAX) {
			objGoal = ObjectiveGoal.MIN;
			for (int i = 0; i < obj.length; i++) {
				obj[i] = -1 * obj[i];
			}
		}

		// Count necessary slack variables (incomplete does not account for bounded
		// variables e.g. 30 >= x <= 80)
		int count = 0;
		for (int i = 0; i < senses.length; i++) {
			if (senses[i] != Sense.EQ) {
				count++;
			}
		}

		// Resize program to accomodate slack variables
		double[][] oldConstraints = constraints;
		double[] oldObjectives = obj;
		int oldNumVariables = numVariables;

		// constraints = new double[]

		for (int i = 0; i < senses.length; i++) {
			if (senses[i] == Sense.LEQ) {

			}
		}
		// <=
	}

	public static enum ObjectiveGoal {
		MIN, MAX
	}

	public static enum Sense {
		LEQ, EQ, GEQ
	}
}
