/*
 * Copyright (c) 2010 Eugenio Realini, Mirko Reguzzoni, Cryms sagl - Switzerland. All Rights Reserved.
 *
 * This file is part of goGPS Project (goGPS).
 *
 * goGPS is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3
 * of the License, or (at your option) any later version.
 *
 * goGPS is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with goGPS.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

package org.gogpsproject.parser.rtcm3;

import org.gogpsproject.util.Base64;

public class ConnectionSettings {

	private String host;
	private int port;
	private String pass_base64;
	private String password;
	private String username;
	private String authbase64;
	private String source;

	public ConnectionSettings(String _host, int _port, String _username,
			String _password) {
		super();
		host = _host;
		port = _port;
		password = _password;
		username = _username;
		setAuthbase64(username + ":" + password);
	}

	public String getAuthbase64() {
		return authbase64;
	}

	public String getHost() {
		return host;
	}

	public String getPass_base64() {
		return pass_base64;
	}

	public int getPort() {
		return port;
	}

	public String getSource() {
		return source;
	}

	public void setAuthbase64(String _authbase64) {
		this.authbase64 = Base64.encode(_authbase64);
	}

	public void setHost(String host) {
		this.host = host;
	}

	public void setPass_base64(String pass_base64) {
		this.pass_base64 = pass_base64;
	}

	public void setPort(int port) {
		this.port = port;
	}

	public void setSource(String source) {
		this.source = source;
	}

}
