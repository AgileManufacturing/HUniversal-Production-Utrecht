package beans;

public class ProductBean {
	private int _orderNumber;
	private boolean _isValidOrderNumber;
	private String _errorMessage;
	
	public void setOrderNumber(String orderNumber) {
		try {
			_orderNumber = Integer.parseInt(orderNumber);
			_isValidOrderNumber = true;
			_errorMessage = "";
		} catch(Exception e) {
			_isValidOrderNumber = false;
			_errorMessage = "Invalid orderNumber: " + orderNumber;
		}
	}
	
	public int getOrderNumber() {
		if(_isValidOrderNumber) {
			return _orderNumber;
		} else {
			return -1;
		}
	}
	
	public boolean isValidOrderNumber() {
		return _isValidOrderNumber;
	}
	
	public String getErrorMessage() {
		return _errorMessage;
	}

}
