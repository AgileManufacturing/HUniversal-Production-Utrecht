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
package org.openbbs.blackboard.persistence;

/**
 * May be thrown by implementations of BlackboardMemory in
 * order to indicate problems.
 */
public class BlackboardMemoryException extends RuntimeException
{
   public BlackboardMemoryException() {
      super();
   }

   public BlackboardMemoryException(String message, Throwable cause) {
      super(message, cause);
   }

   public BlackboardMemoryException(String message) {
      super(message);
   }

   public BlackboardMemoryException(Throwable cause) {
      super(cause);
   }

   private static final long serialVersionUID = -6076245270904356897L;
}
