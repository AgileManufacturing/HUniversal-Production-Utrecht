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
package org.openbbs.blackboard.persistence.snapshot;

import java.io.Serializable;

import org.apache.commons.lang.Validate;

/**
 * Serves as a marker in snapshot files. In order to achieve
 * maximum compatibilty with different serialization mechanisms,
 * this is a simple class and not an enum.
 */
class SnapshotMarker implements Serializable
{
   public static final SnapshotMarker NEW_ZONE = new SnapshotMarker("NEW_ZONE");

   private final String name;

   private SnapshotMarker(String name) {
      Validate.notNull(name);
      this.name = name;
   }

   public boolean equals(Object obj) {
      if (!(obj instanceof SnapshotMarker)) {
         return false;
      }

      return this.name.equals(((SnapshotMarker)obj).name);
   }

   public int hashCode() {
      return this.name.hashCode();
   }

   private static final long serialVersionUID = 3249732671648756577L;
}
