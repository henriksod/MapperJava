import java.util.ArrayList;
import java.util.Map;

/**
 * Returns a list of laser echoes.
 * 
 * @author Thomas Johansson
 * 
 *         Updated by Ola Ringdahl 2014-09-10
 *         Updated by Ola Ringdahl 2015-12-16 (added convert2double)         
 * 
 */
public class LaserEchoesResponse implements Response {
    private Map<String, Object> data;

    public void setData(Map<String, Object> data) {
	this.data = data;
    }

    public double[] getEchoes() {
	ArrayList echoes = (ArrayList) data.get("Echoes");

	Object[] list = echoes.toArray();
	double[] result = new double[list.length];
	for (int i = 0; i < result.length; i++) {
	    result[i] = RobotCommunication.convert2Double(list[i]);
	}
	return result;
    }

    public String getPath() {
	return "/lokarria/laser/echoes";
    }

    public long getTimestamp() {
	return (Long) data.get("TimeStamp");
    }

}
