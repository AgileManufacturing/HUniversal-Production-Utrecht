package nl.hu.message;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import com.mongodb.BasicDBObject;

public class MessageBuilder {

	public enum MessageType {
		SET, GET
	}

	public HashMap<String, Object> patterns;

	public MessageBuilder() {
		patterns = new HashMap<String, Object>();
	}

	public void flush() {
		patterns.clear();
	}

	public HashMap buildMessage(MessageType type) {
		switch (type)
		{
		case GET:
			HashMap<String, Object> start = new HashMap<String, Object>();
			HashMap<String, Object> recursiveMap = start;
			for (Map.Entry<String, Object> entry : patterns.entrySet()) {
				String[] subpattern = entry.getKey().split("\\.");

				// check if a value can be returned directly
				if (subpattern.length == 1) {
					recursiveMap.put(subpattern[0], entry.getValue());
				} else {

					for (int i = 0; i < subpattern.length; i++) {
						// if last subpatern is reached return the value
						if (subpattern.length - 1 == i) {
							recursiveMap.put(subpattern[i], entry.getValue());
						} else {
							// if exception is thrown the patern did not exist
							try {
								// create a new map for from the current
								// subpatern
								HashMap<String, Object> map = (HashMap<String, Object>) recursiveMap
										.get(subpattern[i]);
								if (map == null) {
									map = new HashMap<String, Object>();
								}
								recursiveMap.put(subpattern[i], map);
								recursiveMap = map;
							} catch (Exception e) {
							}

						}
					}
					recursiveMap = start;
				}
			}
			return (HashMap) recursiveMap.clone();
		case SET:
			return (HashMap) patterns.clone();
		}
		return null;
	}

	public void add(String pattern, Object value) {
		patterns.put(pattern, value);
	}
}
