/**
 * @license
 *
 * Copyright 2015 Erle Robotics
 * http://erlerobotics.com
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @fileoverview Blocks for Erle-Spider.
 * @author victor@erlerobot.com (Víctor Mayoral Vilches)
 * @author ahcorde@erlerobot.com (Alejandro Hernández Cordero)
 */
'use strict';

goog.provide('Blockly.Python.code');
goog.require('Blockly.Python');


Blockly.Python['run_code'] = function(block) {
    var code_text = block.getFieldValue('CODE_TEXT');
    var code = "";
	code+=""+code_text+"\n"
	code+="\n"
	return code;
};
