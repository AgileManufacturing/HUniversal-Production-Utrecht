package data;

public class ProductionStep {

	private Long id;
	private String colorCode;
	private int shapeCode;

	private Location location;

	public void setId(Long id) {
		this.id = id;
	}

	public void setColorCode(String c) {
		this.colorCode = c;
	}

	public void setShape(int s) {
		this.shapeCode = s;
	}

	public Long getId() {
		return id;
	}

	public String getColorCode() {
		return colorCode;
	}

	public int getShape() {
		return shapeCode;
	}

	public void setLocation(Location loc) {
		this.location = loc;
	}

	public Location getLocation() {
		return location;
	}

	public static class Location {
		private int row;
		private int column;

		public void setRow(int row) {
			this.row = row;
		}

		public void setColumn(int column) {
			this.column = column;
		}

		public int getRow() {
			return row;
		}

		public int getColumn() {
			return column;
		}

	}

	public String toString() {
		return "";
	}

}
