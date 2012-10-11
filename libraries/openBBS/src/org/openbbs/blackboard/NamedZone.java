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

import org.apache.commons.lang.Validate;
import org.apache.commons.lang.builder.HashCodeBuilder;

/**
 * @author stefan
 */
public class NamedZone implements Zone
{
   private String name = null;

   public NamedZone(String name) {
      Validate.notNull(name);
      this.name = name;
   }

   public String name() {
      return this.name;
   }

   public boolean equals(Object obj) {
      if (obj == null || !(obj instanceof NamedZone)) return false;

      return this.name().equals(((NamedZone)obj).name());
   }

   public int hashCode() {
      return new HashCodeBuilder().append(this.name).append(NamedZone.class).toHashCode();
   }

   public String toString() {
      return "a NamedZone \"" + this.name() + "\"";
   }

   private static final long serialVersionUID = 6286217725870494454L;
}
