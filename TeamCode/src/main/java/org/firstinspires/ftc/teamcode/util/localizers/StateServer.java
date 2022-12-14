package org.firstinspires.ftc.teamcode.util.localizers;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class StateServer extends NanoHTTPD {
    public String state;
    public JSONArray response;
    public int port = 8083;
    /**
     * Constructs an HTTP server on given port.
     *
     * @param
     */
    public StateServer() throws IOException {
        super(8079);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT,false);
        response = new JSONArray();
    }

    public String getResponse(){
        String retval = response.toString();
        response = new JSONArray();
        return retval;
    }


    public void addState(JSONObject obj){
        response.put(obj);
    }

    /**
     * Override this to customize the server.
     * <p>
     * (By default, this returns a 404 "Not Found" plain text error response.)
     *
     * @param session The HTTP session
     * @return HTTP response, see class Response for details
     */
    @Override
    public Response serve(IHTTPSession session) {
        return newFixedLengthResponse(Response.Status.OK, "application/json", getResponse());
    }
}
