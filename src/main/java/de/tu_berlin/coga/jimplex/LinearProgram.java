package de.tu_berlin.coga.jimplex;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.ParseException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import de.tu_berlin.coga.jimplex.LPReader;

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
	int numSlackVariables;

	double[] variables;

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
		numSlackVariables = 0;
		variables = new double[numVariables];
	}

	public int getNumOriginalVariables() {
		return this.numVariables - this.numSlackVariables;
	}

	public int getNumVariables() {
		return this.numVariables;
	}

	public int getNumSlackVariables() {
		return this.numSlackVariables;
	}
	
	public double getObjectiveValue(int[] B) {
		double value = 0;		
		for (int i : B) {
			value += (obj[i] * variables[i]);
		}		
		return value;
	}

	/**
	 * Transforms the linear program in general form to a minimizing linear program in standard form.
	 * <ul>
	 * <li>All variables are transformed to be non-negative restricted</li>
	 * <li>All inequality constraints are transformed to equality constraints</li>
	 * <li>All constants on the ride hand side are transformed to non-negative constants</li>
	 * </ul>
	 */
	public void normalize() {
		// Make all problems minimizing
		if (objGoal == ObjectiveGoal.MAX) {
			objGoal = ObjectiveGoal.MIN;
			for (int i = 0; i < obj.length; i++) {
				obj[i] = -1 * obj[i];
			}
		}
		
		// transform constraints were the constant rhs is less than 0
		for (int i = 0; i < rightHandSide.length; i++) {
			double rhs = rightHandSide[i];
			if (rhs < 0) {
				rightHandSide[i] = -1 * rhs;
				for (double coef : constraints[i]) {
					coef *= -1;
				}
			}
		}

		// Count necessary slack variables (incomplete does not account for bounded
		// variables e.g. 30 >= x <= 80)
		int count = 0;
		List<Integer> slackIndices = new ArrayList<Integer>(constraints.length);
		Map<Integer, Integer> slackMap = new HashMap<Integer, Integer>(constraints.length);

		for (int i = 0; i < senses.length; i++) {
			if (senses[i] != Sense.EQ) {
				slackIndices.add(Integer.valueOf(i));
				slackMap.put(count++, i);
			}
		}

		// Resize program to accommodate slack and excess variables
		double[][] oldConstraints = constraints;
		double[] oldObjectives = obj;
		String[] oldVarNames = varName;
		double[] oldVars = variables;
		double[] oldLBounds = lowerBound;
		double[] oldUBounds = upperBound;

		numSlackVariables = count;
		int numVarsTotal = numVariables + numSlackVariables;
		obj = new double[numVarsTotal];
		varName = new String[numVarsTotal];
		variables = new double[numVarsTotal];
		lowerBound = new double[numVarsTotal];
		upperBound = new double[numVarsTotal];

		constraints = new double[constraints.length][numVarsTotal];

		System.arraycopy(oldObjectives, 0, obj, 0, numVariables);
		System.arraycopy(oldVarNames, 0, varName, 0, numVariables);
		System.arraycopy(oldVars, 0, variables, 0, numVariables);
		System.arraycopy(oldLBounds, 0, lowerBound, 0, numVariables);
		System.arraycopy(oldUBounds, 0, upperBound, 0, numVariables);

		for (int i = 0; i < constraints.length; i++) {
			System.arraycopy(oldConstraints[i], 0, constraints[i], 0, numVariables);
		}

		for (Entry<Integer, Integer> slackMapping : slackMap.entrySet()) {
			int slackNum = slackMapping.getKey();
			int constNum = slackMapping.getValue();
			if (senses[constNum] == Sense.LEQ) {
				varName[numVariables + slackNum] = "s" + (constNum + 1);
			} 
			else {
				varName[numVariables + slackNum] = "e" + (constNum + 1);
			}
			constraints[constNum][numVariables + slackNum] = 1.0;
			lowerBound[numVariables + slackNum] = 0;
			upperBound[numVariables + slackNum] = Double.POSITIVE_INFINITY;
			senses[constNum] = Sense.EQ;
		}

		numVariables = numVarsTotal;
		
		// Transform restricted ( != non-negative) and free variables to constraints and non-negative variables
		for (int i = 0; i < lowerBound.length; i++) {
			double lb = lowerBound[i];
			double ub = upperBound[i];
			if (lb != 0 || ub < Double.POSITIVE_INFINITY) {
				// Transform unrestricted / free variables
				if (Double.isInfinite(lb) && Double.isInfinite(ub)) {
					// TODO
					
				}
			}
		}
	}

	public static enum ObjectiveGoal {
		MIN, MAX
	}

	public static enum Sense {
		LEQ, EQ, GEQ
	}
}
