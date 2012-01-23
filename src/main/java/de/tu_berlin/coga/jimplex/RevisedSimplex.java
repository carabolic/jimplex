package de.tu_berlin.coga.jimplex;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.text.ParseException;
import java.util.Arrays;

import org.ejml.data.DenseMatrix64F;
import org.ejml.data.RowD1Matrix64F;
import org.ejml.ops.CommonOps;

import de.tu_berlin.coga.jimplex.LinearProgram;
import de.tu_berlin.coga.jimplex.LinearProgram.ObjectiveGoal;
import de.tu_berlin.coga.jimplex.exceptions.InfeasibleLPException;
import de.tu_berlin.coga.jimplex.exceptions.UnboundedLPException;

public class RevisedSimplex {
	private LinearProgram program;

	public RevisedSimplex(LinearProgram program) {
		this.program = program;
		// TODO: Check that it is normalized
	}

	public int[] solve() throws InfeasibleLPException, UnboundedLPException {
		int numVariables = program.numVariables;
		int numConstraints = program.constraints.length;
		double[][] oldConstraints = program.constraints;
		double[] oldCosts = program.obj;
		double[] oldLBounds = program.lowerBound;
		double[] oldUBounds = program.upperBound;

		double[][] newConstraints = new double[numConstraints][numVariables + numConstraints];

		// Write identity matrix
		for (int i = 0; i < numConstraints; i++) {
			newConstraints[i][i] = 1;
		}

		// Copy old constraints
		for (int i = 0; i < numConstraints; i++) {
			for (int j = 0; j < numVariables; j++) {
				newConstraints[i][numConstraints + j] = program.constraints[i][j];
			}
		}

		double[] newCosts = new double[numVariables + numConstraints];
		for (int i = 0; i < numConstraints; i++) {
			newCosts[i] = 1;
		}

		double[] newLBounds = new double[numVariables + numConstraints];
		double[] newUBounds = new double[numVariables + numConstraints];
		for (int i = 0; i < numConstraints; i++) {
			newUBounds[i] = Double.POSITIVE_INFINITY;
		}
		System.arraycopy(oldLBounds, 0, newLBounds, numConstraints, numVariables);
		System.arraycopy(oldUBounds, 0, newUBounds, numConstraints, numVariables);

		// Find base solution
		program.constraints = newConstraints;
		program.obj = newCosts;
		program.numVariables = numVariables + numConstraints;
		program.lowerBound = newLBounds;
		program.upperBound = newUBounds;
		int[] B = new int[numConstraints];
		for (int i = 0; i < B.length; i++) {
			B[i] = i;
		}
		B = solve(B);
		
		if (!isFeasible(B)) { throw new InfeasibleLPException(); }
		

		// Optimize original program using that solution
		program.constraints = oldConstraints;
		program.obj = oldCosts;
		program.numVariables = numVariables;
		program.lowerBound = oldLBounds;
		program.upperBound = oldUBounds;
		for (int i = 0; i < B.length; i++) {
			B[i] = B[i] - numConstraints;
		}
		B = solve(B);

		return B;
	}

	public int[] solve(int[] B) throws UnboundedLPException {
		double[][] A = program.constraints;
		double[] c = program.obj;
		DenseMatrix64F b = convertColumnVector(program.rightHandSide);

		int rank = B.length;
		int numVariables = program.numVariables;
		int numNonBaseColumns = numVariables - rank;

		while (true) {
			double[] variables = new double[program.numVariables];
			DenseMatrix64F A_Binv = createInvBase(A, B);
			DenseMatrix64F b_tilde = new DenseMatrix64F(rank, 1);
			CommonOps.mult(A_Binv, b, b_tilde);

			DenseMatrix64F pi_trans = new DenseMatrix64F(1, rank);
			CommonOps.mult(getBaseCosts(c, B), A_Binv, pi_trans);

			int[] nonBaseColumns = calculateNonBaseColumns(numVariables, B);
			// Calculate reduced costs
			double[] c_tilde = new double[numNonBaseColumns];
			for (int i = 0; i < numNonBaseColumns; i++) {
				int columnIndex = nonBaseColumns[i];
				double[] column = getColumn(A, columnIndex);
				double sum = 0;

				for (int j = 0; j < column.length; j++) {
					sum += pi_trans.get(0, j) * column[j];
				}

				c_tilde[i] = c[columnIndex] - sum;
			}

			// Check termination
			boolean allPositive = true;
			for (int i = 0; i < c_tilde.length; i++) {
				if (c_tilde[i] < 0) {
					allPositive = false;
					break;
				}
			}
			if (allPositive) {
				// Optimal solution
				StringBuilder strBld = new StringBuilder();

				for (int i = 0; i < B.length; i++) {
					double var = b_tilde.get(i);
					if (var <= 0.0) {
						var = 0;
					}
					variables[B[i]] = var;
					// strBld.append(program.varName[B[i]]);
					// strBld.append(": ");
					strBld.append(var);
					strBld.append("\n");
				}
				program.variables = variables;
				System.out.println(strBld.toString());

				return B;
			}

			// Choose pivot (min index rule)
			//   int index = -1;
			//   for (int i = 0; i < c_tilde.length; i++) {
			//    if (c_tilde[i] < 0) {
			//     index = i;
			//     break;
			//    }
			//   }
			int index = -1;
			double min = Double.POSITIVE_INFINITY;
			for (int i = 0; i < c_tilde.length; i++) {
				if (c_tilde[i] < min) {
					min = c_tilde[i];
					index = i;
				}
			}
			int pivotColumn = nonBaseColumns[index];

			// Generate Pivot Column
			double[] column = getColumn(A, pivotColumn);
			DenseMatrix64F entering_column = convertColumnVector(column);
			DenseMatrix64F weight_vector = new DenseMatrix64F(rank, 1);
			CommonOps.mult(A_Binv, entering_column, weight_vector);

			// Find pivot element
			double min_value = Double.POSITIVE_INFINITY;
			int min_index = -1;
			for (int i = 0; i < weight_vector.getNumRows(); i++) {
				double x_is = weight_vector.get(i, 0);
				if (x_is > 0) {
					double value = b_tilde.get(i, 0) / x_is;
					if (value < min_value) {
						min_value = value;
						min_index = i;
					}
				}
			}

			// Check if problem is unbound
			if (min_value == Double.POSITIVE_INFINITY) {
				throw new UnboundedLPException();
			}

			int k = B[min_index];
			B[min_index] = pivotColumn;
			for (int i = 0; i < numNonBaseColumns; i++) {
				if (nonBaseColumns[i] == pivotColumn) {
					nonBaseColumns[i] = k;
					break;
				}
			}
			// System.out.println(B[0] + ", " + B[1]);
			// System.out.println(b_tilde);
		}
	}

	private boolean isFeasible(int[] B) {
		boolean feasible = true;
		for (int i : B) {
			if (i < B.length) {
				return false;
			}
		}

		return feasible;
	}

	private double[] getColumn(double[][] A, int columnIndex) {
		double[] column = new double[A.length];

		for (int i = 0; i < column.length; i++) {
			column[i] = A[i][columnIndex];
		}

		return column;
	}

	private int[] calculateNonBaseColumns(int numVariables, int[] b) {
		int[] tmp = new int[numVariables];
		for (int i = 0; i < b.length; i++) {
			tmp[b[i]] = 1;
		}

		int[] nonBaseColumns = new int[numVariables - b.length];
		int j = 0;
		for (int i = 0; i < numVariables; i++) {
			if (tmp[i] == 0) {
				nonBaseColumns[j] = i;
				j++;
			}
		}

		return nonBaseColumns;
	}

	private RowD1Matrix64F getBaseCosts(double[] c, int[] B) {
		double[] c_base = new double[B.length];
		for (int i = 0; i < B.length; i++) {
			c_base[i] = c[B[i]];
		}

		return convertRowVector(c_base);
	}

	private DenseMatrix64F convertRowVector(double[] vector) {
		DenseMatrix64F vectorMatrix = new DenseMatrix64F(1, vector.length);
		for (int i = 0; i < vector.length; i++) {
			vectorMatrix.set(0, i, vector[i]);
		}

		return vectorMatrix;
	}

	private DenseMatrix64F convertColumnVector(double[] vector) {
		DenseMatrix64F vectorMatrix = new DenseMatrix64F(vector.length, 1);
		for (int i = 0; i < vector.length; i++) {
			vectorMatrix.set(i, 0, vector[i]);
		}

		return vectorMatrix;
	}

	private DenseMatrix64F createInvBase(double[][] A, int[] B) {
		int rows = A.length;
		double[][] A_B = new double[rows][B.length];
		for (int i = 0; i < B.length; i++) {
			for (int j = 0; j < rows; j++) {
				A_B[j][i] = A[j][B[i]];
			}
		}

		DenseMatrix64F matrix = new DenseMatrix64F(A_B);
		CommonOps.invert(matrix);
		return matrix;
	}

	public static void main(String[] args) throws FileNotFoundException, ParseException, IOException,
			InfeasibleLPException, UnboundedLPException {
		String filePath = args[0];
		LinearProgram program = new LinearProgram(filePath);
		boolean isMax = (program.objGoal == ObjectiveGoal.MAX) ? true : false;
		program.normalize();
		System.out.println(program);
		boolean isConverted = isMax && (program.objGoal == ObjectiveGoal.MIN);

		RevisedSimplex solver = new RevisedSimplex(program);
		int[] opt = solver.solve();
		Arrays.sort(opt);

		double value = program.getObjectiveValue(opt);
		if (isConverted) {
			value *= -1;
		}
		System.out.println("Objective: " + value);

		for (int i : opt) {
			System.out.println(program.varName[i] + ": " + program.variables[i]);
		}
	}
}
