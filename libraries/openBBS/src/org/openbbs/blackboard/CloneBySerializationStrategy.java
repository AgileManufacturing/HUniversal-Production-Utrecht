/*
 *  Copyright [2006-2007] [Stefan Kleine Stegemann]
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */
package org.openbbs.blackboard;

import java.io.Serializable;

import org.apache.commons.lang.SerializationUtils;
import org.apache.commons.lang.Validate;

/**
 * An object is (deep-)cloned by creating serialization and de-serialization.
 * Requires that objects which are cloned with this strategy implement the
 * Serializable interface.
 * 
 * @author stefan
 */
public class CloneBySerializationStrategy implements CloneStrategy
{
   public Object clone(Object obj) {
      if (obj == null) return null;

      Validate.isTrue(obj instanceof Serializable, "object " + obj.toString() + " is not serializable");
      return SerializationUtils.clone((Serializable)obj);
   }
}
