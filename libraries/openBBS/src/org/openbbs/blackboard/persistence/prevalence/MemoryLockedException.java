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
package org.openbbs.blackboard.persistence.prevalence;

/**
 * Throw when someone tries to read or modify a locked memory. A memory is
 * usually locked only for a limited amount of time, so it's worth to retry
 * the operation later.
 */
public class MemoryLockedException extends PrevalencePersistenceException
{

   public MemoryLockedException() {
      super();
   }

   public MemoryLockedException(String message, Throwable cause) {
      super(message, cause);
   }

   public MemoryLockedException(String message) {
      super(message);
   }

   public MemoryLockedException(Throwable cause) {
      super(cause);
   }

   private static final long serialVersionUID = -4559949109133317574L;
}
