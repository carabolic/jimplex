package de.tu_berlin.coga.jimplex;

import java.io.*;
import java.util.*;
import java.text.ParseException;

/**
 * A class to read linear programs from <code>.lp</code> files.
 * 
 * Initialise with the file name and invoke the method <code>readLP()</code> to
 * parse an lp from an <code>.lp</code> file. After successful reading all data
 * is held in various arrays which can be accessed via class methods. Debug
 * output can be switched on by an additional parameter to the constructor. <br/>
 * Note that the parser cannot yet handle integer and binary variables!
 */
public class LPReader {
	String filename;
	boolean verbose;

	final int SEC_START = 0;
	final int SEC_OBJECTIVE = 1;
	final int SEC_CONSTRAINTS = 2;
	final int SEC_BOUNDS = 3;
	final int SEC_END = 4;

	/** represents <= */
	public static final int SENSE_LEQ = -1;
	/** represents = */
	public static final int SENSE_EQ = 0;
	/** represents >= */
	public static final int SENSE_GEQ = 1;
	/** represents an undefined sense */
	public static final int SENSE_UNDEF = -17;
	/** represents minimising the objective */
	public static final int SENSE_MIN = -2;
	/** represents maximising the objective */
	public static final int SENSE_MAX = 2;

	final String VARNAME_PATTERN = "[a-zA-Z\\!\"#\\$%&\\(\\)/,;\\?`'\\{\\}\\|~_][a-zA-Z0-9\\!\"#\\$%&\\(\\)/,\\.;\\?`'\\{\\}\\|~_]*";

	Constraint objRaw;
	HashMap<String, Constraint> constrHash;
	HashMap<String, Variable> varHash;

	double[][] constraint;
	int[] sense;
	double[] rhs;
	String[] constrName;
	double[] obj;
	int objsense;
	double[] lbound;
	double[] ubound;
	String[] varName;

	protected class Constraint {
		public String name;
		public int no;
		public HashMap<String, Coefficient> coeff;
		public int sense;
		public double rhs;

		public Constraint(String n, int i) {
			name = n;
			no = i;
			coeff = new HashMap<String, Coefficient>();
			sense = SENSE_UNDEF;
			rhs = 0;
			logprint("new Constraint '" + name + "' (no " + i + ")");
		}
	}

	protected class Variable {
		public String name;
		public int no;
		public double lb;
		public double ub;

		public Variable(String n, int i) {
			name = n;
			no = i;
			lb = 0;
			ub = Double.POSITIVE_INFINITY;
			logprint("new Variable '" + name + "' (no " + i + ")");
		}
	}

	protected class Coefficient {
		public int varNo;
		public int constrNo;
		public double value;

		public Coefficient(int var, int constr, double val) {
			varNo = var;
			constrNo = constr;
			value = val;
		}
	}

	/**
	 * Initialises the parser with no log output.
	 * 
	 * @param fname
	 *          file name to read from
	 */
	public LPReader(String fname) {
		this(fname, false);
	}

	/**
	 * Initialises the parser.
	 * 
	 * @param fname
	 *          file name to read from
	 * @param verb
	 *          switch log output
	 */
	public LPReader(String fname, boolean verb) {
		filename = fname;
		verbose = verb;

		constraint = new double[0][0];
		sense = new int[0];
		rhs = new double[0];
		constrName = new String[0];
		obj = new double[0];
		objsense = SENSE_UNDEF;
		lbound = new double[0];
		ubound = new double[0];
		varName = new String[0];
	}

	/**
	 * Tells the number of variables.
	 * 
	 * @return the number of variables of the linear program read
	 */
	public int noOfVariables() {
		return varName.length;
	}

	/**
	 * Tells the number of constraints.
	 * 
	 * @return the number of constraints of the linear program read
	 */
	public int noOfConstraints() {
		return constrName.length;
	}

	/**
	 * Tells the constraints coefficient matrix.
	 * 
	 * @return the lhs coefficients of all constraints as an array of size
	 *         <code>noOfConstraints()</code> &times; <code>noOfVariables()</code>
	 */
	public double[][] constraintsMatrix() {
		return constraint;
	}

	/**
	 * Tells the senses of the constraints.
	 * 
	 * @return the senses of the constraints as an array of size
	 *         <code>noOfConstraints()</code>
	 */
	public int[] senseVector() {
		return sense;
	}

	/**
	 * Tells the right hand side coefficients of the constraints.
	 * 
	 * @return the rhs of the constraints as an array of size
	 *         <code>noOfConstraints()</code>
	 */
	public double[] rhsVector() {
		return rhs;
	}

	/**
	 * Tells the names of the constraints.
	 * 
	 * @return the names of the constraints as an array of size
	 *         <code>noOfConstraints()</code>
	 */
	public String constraintName(int i) {
		return constrName[i];
	}

	/**
	 * Tells the coefficients of the objective.
	 * 
	 * @return the coefficients of the objective as an array of size
	 *         <code>noOfVariables()</code>
	 */
	public double[] objectiveVector() {
		return obj;
	}

	/**
	 * Tells whether the objective is to be maximised or minimised.
	 * 
	 * @return <code>SENSE_MAX</code> of <code>SENSE_MIN</code>
	 */
	public int objectiveSense() {
		return objsense;
	}

	/**
	 * Tells the lower bounds of the variables.
	 * 
	 * @return the lower bounds of the variables as an array of size
	 *         <code>noOfVariables()</code>
	 */
	public double[] lowerBoundVector() {
		return lbound;
	}

	/**
	 * Tells the upper bounds of the variables.
	 * 
	 * @return the upper bounds of the variables as an array of size
	 *         <code>noOfVariables()</code>
	 */
	public double[] upperBoundVector() {
		return ubound;
	}

	/**
	 * Tells the names of the variables.
	 * 
	 * @return the names of the variables as an array of size
	 *         <code>noOfVariables()</code>
	 */
	public String variableName(int j) {
		return varName[j];
	}

	/**
	 * Attempts to read a linear program from the file with which the parser was
	 * initialised.
	 */
	public void readLP() throws ParseException, FileNotFoundException, IOException {
		BufferedReader in = new BufferedReader(new FileReader(filename));

		constrHash = new HashMap<String, Constraint>();
		varHash = new HashMap<String, Variable>();
		logprint("switching to status SEC_START");
		int status = SEC_START;
		Constraint curConstr = null;
		int constrNo = 0;
		int lineNo = 0;
		boolean constrComplete = true;
		while ((in.ready()) && (status != SEC_END)) {
			String line = in.readLine();
			++lineNo;
			logprint("----------------------------------------------------\n" + "line " + lineNo + ": " + line + "\n"
					+ "----------------------------------------------------");
			int commentStart = line.indexOf('\\');
			if (commentStart != -1)
				line = line.substring(0, commentStart);
			line = line.trim();

			int colonIndex = line.indexOf(':');
			if (colonIndex >= 0) {
				logprint("parsing name");
				String namePart = line.substring(0, colonIndex).trim();
				switch (status) {
				case SEC_START:
					throw new ParseException("line " + lineNo + ": unexpected ':' before objective section", lineNo);
				case SEC_OBJECTIVE: {
					if (!namePart.matches(VARNAME_PATTERN))
						throw new ParseException("line " + lineNo + ": invalid objective name '" + namePart + "'", lineNo);
					logprint("setting objective name '" + namePart + "'");
					curConstr.name = namePart;
				}
					break;
				case SEC_CONSTRAINTS: {
					if (!namePart.matches(VARNAME_PATTERN))
						throw new ParseException("line " + lineNo + ": invalid constraint name '" + namePart + "'", lineNo);
					logprint("setting constraint name '" + namePart + "'");
					curConstr.name = namePart;
				}
					break;
				case SEC_BOUNDS:
					throw new ParseException("line " + lineNo + ": unexpected ':' in bounds section", lineNo);
				}
				line = line.substring(colonIndex + 1).trim();
			}

			if (line.length() > 0) {
				String keyword = line.toLowerCase();
				if (keyword.equals("end")) {
					if (constrComplete) {
						logprint("switching to status SEC_END");
						status = SEC_END;
					} else
						throw new ParseException("line " + lineNo + ": incomplete constraint", lineNo);
				} else {
					switch (status) {
					case SEC_START: {
						curConstr = new Constraint("obj", 0);
						if (keyword.equals("max") || keyword.equals("maximize") || keyword.equals("maximise")) {
							logprint("recognised a maximising problem");
							curConstr.sense = SENSE_MAX;
						} else if (keyword.equals("min") || keyword.equals("minimize") || keyword.equals("minimise")) {
							logprint("recognised a minimising problem");
							curConstr.sense = SENSE_MIN;
						} else
							throw new ParseException("line " + lineNo + ": unrecognised keyword '" + keyword + "'", lineNo);
						logprint("switching to status SEC_OBJECTIVE");
						status = SEC_OBJECTIVE;
					}
						break;
					case SEC_OBJECTIVE: {
						if (keyword.equals("subject to") || keyword.equals("such that") || keyword.equals("s.t.")
								|| keyword.equals("st.") || keyword.equals("st")) {
							logprint("saving objective");
							objRaw = curConstr;
							logprint("switching to status SEC_CONSTRAINTS");
							status = SEC_CONSTRAINTS;
							logprint("initialising new Constraint");
							curConstr = new Constraint("c" + constrNo, constrNo);
						} else {
							logprint("parsing linear combination '" + line + "'");
							parseLinComb(line, curConstr, lineNo);
						}
					}
						break;
					case SEC_CONSTRAINTS: {
						if (keyword.equals("bounds") || keyword.equals("bound")) {
							if (constrComplete) {
								logprint("switching to status SEC_BOUNDS");
								status = SEC_BOUNDS;
							} else
								throw new ParseException("line " + lineNo + ": incomplete constraint", lineNo);
						} else {
							logprint("parsing linear combination '" + line + "'");
							constrComplete = parseLinComb(line, curConstr, lineNo);
							if (constrComplete) {
								logprint("constraint is complete");
								if (constrHash.containsKey(curConstr.name)) {
									String addInfo = "'";
									if (curConstr.name.charAt(0) == 'c')
										addInfo += " (maybe an earlier constraint got that name automatically)";
									throw new ParseException("line " + lineNo + ": ambiguous constraint name '" + curConstr.name
											+ addInfo, lineNo);
								}
								logprint("saving constraint '" + curConstr.name + "'");
								constrHash.put(curConstr.name, curConstr);
								++constrNo;
								logprint("initialising new Constraint");
								curConstr = new Constraint("c" + constrNo, constrNo);
							}
						}
					}
						break;
					case SEC_BOUNDS: {
						logprint("parsing bounds");
						parseBound(line, lineNo);
					}
						break;
					}
				}
			}
		}
		logprint("====================================================\nwriting to arrays");
		hash2arr();
	}

	private void parseBound(String expr, int lineNo) throws ParseException {
		String[] exprsplit = expr.split("\\<\\=");
		switch (exprsplit.length) {
		case 1: {
			exprsplit = expr.split("=");
			if (exprsplit.length > 1) {
				logprint("..equality bound");
				String varname = exprsplit[0].trim();
				Variable var = varHash.get(varname);
				if (var == null)
					throw new ParseException("line " + lineNo + ": unknown variable '" + varname + "'", lineNo);
				String boundStr = exprsplit[1].trim();
				double bound = 0;
				try {
					bound = Double.parseDouble(boundStr);
				} catch (NumberFormatException ex) {
					throw new ParseException("line " + lineNo + ": '" + boundStr + "' is not a valid number", lineNo);
				}
				var.lb = bound;
				var.ub = bound;
			} else {
				logprint("..free variable");
				exprsplit = expr.split("\\s");
				if (exprsplit.length < 2)
					throw new ParseException("line " + lineNo + ": illegal free variable expression", lineNo);
				if (!exprsplit[1].trim().toLowerCase().equals("free"))
					throw new ParseException("line " + lineNo + ": expected 'free'", lineNo);
				String varname = exprsplit[0].trim();
				Variable var = varHash.get(varname);
				if (var == null)
					throw new ParseException("line " + lineNo + ": unknown variable '" + varname + "'", lineNo);
				var.lb = Double.NEGATIVE_INFINITY;
				var.ub = Double.POSITIVE_INFINITY;
			}
		}
			break;
		case 2: {
			logprint("..one-sided bound");
			String varname = exprsplit[0].trim();
			String boundStr = exprsplit[1].trim();
			int sense = SENSE_LEQ;
			Variable var = varHash.get(varname);
			if (var == null) {
				varname = exprsplit[1].trim();
				boundStr = exprsplit[0].trim();
				sense = SENSE_GEQ;
				var = varHash.get(varname);
				if (var == null)
					throw new ParseException("line " + lineNo + ": neither '" + boundStr + "' nor '" + varname
							+ "' is a known variable", lineNo);
			}
			double bound = 0;
			try {
				bound = Double.parseDouble(boundStr);
			} catch (NumberFormatException ex) {
				throw new ParseException("line " + lineNo + ": '" + boundStr + "' is not a valid bound", lineNo);
			}
			switch (sense) {
			case SENSE_LEQ:
				logprint("..upper");
				var.ub = bound;
				break;
			case SENSE_GEQ:
				logprint("..lower");
				var.lb = bound;
				break;
			}
		}
			break;
		case 3: {
			logprint("..two-sided bound");
			String varname = exprsplit[1].trim();
			Variable var = varHash.get(varname);
			if (var == null)
				throw new ParseException("line " + lineNo + ": unknown variable '" + varname + "'", lineNo);
			double lower = 0;
			String lowerStr = exprsplit[0].trim().toLowerCase();
			if (lowerStr.equals("-infinity") || lowerStr.equals("-inf"))
				lower = Double.NEGATIVE_INFINITY;
			else {
				try {
					lower = Double.parseDouble(lowerStr);
				} catch (NumberFormatException ex) {
					throw new ParseException("line " + lineNo + ": '" + lowerStr + "' is not a valid bound", lineNo);
				}
			}
			double upper = 0;
			String upperStr = exprsplit[2].trim().toLowerCase();
			if (upperStr.equals("infinity") || upperStr.equals("inf") || upperStr.equals("+infinity")
					|| upperStr.equals("+inf"))
				upper = Double.POSITIVE_INFINITY;
			else {
				try {
					upper = Double.parseDouble(exprsplit[2].trim());
				} catch (NumberFormatException ex) {
					throw new ParseException("line " + lineNo + ": '" + exprsplit[2].trim() + "' is not a valid bound", lineNo);
				}
			}
			var.lb = lower;
			var.ub = upper;
		}
		}
	}

	private boolean parseLinComb(String expr, Constraint target, int lineNo) throws ParseException {
		StringTokenizer exprtok = new StringTokenizer(expr, "+->=<", true);
		boolean constrComplete = false;
		int currSign = 1;
		boolean inequflag = false;
		while (exprtok.hasMoreTokens()) {
			String nextToken = exprtok.nextToken().trim();
			if (nextToken.length() > 0) {
				if (nextToken.equals("+")) {
					logprint("..sign is +");
					currSign = 1;
				} else if (nextToken.equals("-")) {
					logprint("..sign is -");
					currSign = -1;
				} else if (nextToken.equals(">")) {
					logprint("..sense is >=");
					target.sense = SENSE_GEQ;
					inequflag = true;
					currSign = 1;
				} else if (nextToken.equals("<")) {
					logprint("..sense is <=");
					target.sense = SENSE_LEQ;
					inequflag = true;
					currSign = 1;
				} else if (nextToken.equals("=")) {
					if (target.sense == SENSE_UNDEF) {
						logprint("..sense is =");
						target.sense = SENSE_EQ;
					} else if (!inequflag)
						throw new ParseException("line " + lineNo + ": illegal '='", lineNo);
					currSign = 1;
				} else {
					if (currSign == 0)
						throw new ParseException("line " + lineNo + ": missing sign", lineNo);
					if ((target.sense == SENSE_GEQ) || (target.sense == SENSE_EQ) || (target.sense == SENSE_LEQ)) {
						logprint("..parsing rhs");
						try {
							target.rhs = Double.parseDouble(nextToken);
						} catch (NumberFormatException ex) {
							throw new ParseException("line " + lineNo + ": '" + nextToken + "' is not a valid rhs entry", lineNo);
						}
						logprint("..saving rhs in Constraint");
						target.rhs *= currSign;
						constrComplete = true;
					} else {
						logprint("..parsing summand");
						parseSummand(nextToken, target, lineNo, currSign);
						currSign = 0;
					}
				}
			}
		}

		return constrComplete;
	}

	private void parseSummand(String expr, Constraint target, int lineNo, int sign) throws ParseException {
		int si = summandSplitIndex(expr);
		if (si == -1)
			throw new ParseException("line " + lineNo + ": '" + expr + "' is not a valid summand", lineNo);
		double coeff = 1;
		if (si > 0) {
			String coeffStr = expr.substring(0, si).trim();
			try {
				coeff = Double.parseDouble(coeffStr);
			} catch (NumberFormatException ex) {
				throw new ParseException("line " + lineNo + ": '" + coeffStr + "' is not a valid number", lineNo);
			}
		}

		String varStr = expr.substring(si).trim();
		if (!varStr.matches(VARNAME_PATTERN))
			throw new ParseException("line " + lineNo + ": '" + varStr + "' is not a valid variable name", lineNo);
		Variable var = varHash.get(varStr);
		if (var == null) {
			var = new Variable(varStr, varHash.size());
			varHash.put(varStr, var);
			target.coeff.put(varStr, new Coefficient(var.no, target.no, sign * coeff));
		} else {
			Coefficient co = target.coeff.get(varStr);
			if (co == null) {
				target.coeff.put(varStr, new Coefficient(var.no, target.no, sign * coeff));
			} else {
				co.value += coeff;
			}
		}
	}

	private int summandSplitIndex(String expr) {
		boolean inExp = false;
		for (int i = 0; i < expr.length(); ++i) {
			char c = expr.charAt(i);
			if (!(Character.isDigit(c) || (c == '.'))) {
				if ((c == 'e') || (c == 'E')) {
					if (inExp)
						return i;
					else
						inExp = true;
				} else
					return i;
			}
		}
		return -1;
	}

	private void logprint(String what) {
		if (verbose)
			System.out.println(what);
	}

	private void hash2arr() {
		int noOfConstr = constrHash.size();
		int noOfVar = varHash.size();

		logprint(noOfConstr + " constraints, " + noOfVar + " variables");
		constraint = new double[noOfConstr][noOfVar];
		for (int i = 0; i < noOfConstr; ++i)
			for (int j = 0; j < noOfVar; ++j)
				constraint[i][j] = 0;
		sense = new int[noOfConstr];
		rhs = new double[noOfConstr];
		constrName = new String[noOfConstr];
		obj = new double[noOfVar];
		for (int j = 0; j < noOfVar; ++j)
			obj[j] = 0;
		objsense = objRaw.sense;
		lbound = new double[noOfVar];
		ubound = new double[noOfVar];
		varName = new String[noOfVar];

		for (Iterator<Coefficient> objCoeffIt = objRaw.coeff.values().iterator(); objCoeffIt.hasNext();) {
			Coefficient coeff = objCoeffIt.next();
			obj[coeff.varNo] = coeff.value;
		}

		for (Iterator<Constraint> constrIt = constrHash.values().iterator(); constrIt.hasNext();) {
			Constraint curConstr = constrIt.next();
			int i = curConstr.no;
			sense[i] = curConstr.sense;
			rhs[i] = curConstr.rhs;
			constrName[i] = curConstr.name;
			for (Iterator<Coefficient> coeffIt = curConstr.coeff.values().iterator(); coeffIt.hasNext();) {
				Coefficient coeff = coeffIt.next();
				constraint[i][coeff.varNo] = coeff.value;
			}
		}

		for (Iterator<Variable> varIt = varHash.values().iterator(); varIt.hasNext();) {
			Variable curVar = varIt.next();
			int j = curVar.no;
			lbound[j] = curVar.lb;
			ubound[j] = curVar.ub;
			varName[j] = curVar.name;
		}
	}

}
