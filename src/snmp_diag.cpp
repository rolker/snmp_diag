#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

struct snmp_session session, *ss;
std::string host = "localhost";
ros::Publisher diag_pub;

void timerCallback(const ros::TimerEvent event)
{
  struct snmp_pdu *pdu;
  struct snmp_pdu *response;
  pdu = snmp_pdu_create(SNMP_MSG_GET);

  oid anOID[MAX_OID_LEN];
  size_t anOID_len = MAX_OID_LEN;

  get_node("ifInOctets.8", anOID, &anOID_len);
  snmp_add_null_var(pdu, anOID, anOID_len);
  
  int status = snmp_synch_response(ss, pdu, &response);
  if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
  {
    diagnostic_msgs::DiagnosticStatus ds;
    ds.name = "snmp";
    ds.hardware_id = host;
    diagnostic_msgs::KeyValue kv;

    for(variable_list* vars = response->variables; vars; vars = vars->next_variable)
    {
      print_variable(vars->name, vars->name_length, vars);
      if(vars->type == ASN_COUNTER)
      {
        kv.key = "ifInOctets.8";
        kv.value = std::to_string(*vars->val.integer);
        ds.values.push_back(kv);
      }
    }
    diagnostic_msgs::DiagnosticArray da;
    da.status.push_back(ds);
    diag_pub.publish(da);
  }

  if (response)
    snmp_free_pdu(response);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snmp_diag");

  host = ros::param::param<std::string>("~host", host);

  init_snmp("snmp_diag");
  snmp_sess_init( &session ); 
  session.peername = strdup(host.c_str());

  session.version = SNMP_VERSION_2c;
  std::string community("public");
  session.community = reinterpret_cast<u_char*>(strdup(community.c_str()));
  session.community_len = community.size();
  
  ss = snmp_open(&session); 
  if (!ss) {
    snmp_perror("ack");
    snmp_log(LOG_ERR, "something horrible happened!!!\n");
    exit(2);
  }

  ros::NodeHandle nh;

  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);

  diag_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

  ros::spin();
  snmp_close(ss);
  return 0;
}
