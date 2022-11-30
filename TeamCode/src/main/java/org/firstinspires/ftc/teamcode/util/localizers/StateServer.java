package org.firstinspires.ftc.teamcode.util.localizers;

import java.io.IOException;

import fi.iki.elonen.NanoHTTPD;

public class StateServer extends NanoHTTPD {
    public String state;
    /**
     * Constructs an HTTP server on given port.
     *
     * @param
     */
    public StateServer() throws IOException {
        super(8083);
        start(NanoHTTPD.SOCKET_READ_TIMEOUT,false);
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
        return newFixedLengthResponse(state);
    }
}
