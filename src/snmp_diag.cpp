#include <net-snmp/net-snmp-config.h>
#include <net-snmp/net-snmp-includes.h>

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>

struct snmp_session session, *ss;
std::string host = "localhost";
std::vector<std::string> oids;
ros::Publisher diag_pub;

void timerCallback(const ros::TimerEvent event)
{

  diagnostic_msgs::DiagnosticArray da;
  da.header.stamp = ros::Time::now();
  diagnostic_msgs::DiagnosticStatus ds;

  ds.name = "snmp";
  ds.hardware_id = host;
  ds.level = diagnostic_msgs::DiagnosticStatus::WARN;

  for(auto oid_str: oids)
  {
    snmp_pdu *pdu = snmp_pdu_create(SNMP_MSG_GET);
    oid anOID[MAX_OID_LEN];
    size_t anOID_len = MAX_OID_LEN;

    get_node(oid_str.c_str(), anOID, &anOID_len);
    snmp_add_null_var(pdu, anOID, anOID_len);

    struct snmp_pdu *response = nullptr;
    int status = snmp_synch_response(ss, pdu, &response);
    if (status == STAT_SUCCESS && response->errstat == SNMP_ERR_NOERROR)
    {

      for(variable_list* vars = response->variables; vars; vars = vars->next_variable)
      {
        print_variable(vars->name, vars->name_length, vars);
        diagnostic_msgs::KeyValue kv;
        kv.key = oid_str;
        switch(vars->type)
        {
          case ASN_INTEGER:
          case ASN_COUNTER:
          case ASN_GAUGE:
          case ASN_TIMETICKS:
            kv.value = std::to_string(*vars->val.integer);
            ds.values.push_back(kv);
            ds.level = diagnostic_msgs::DiagnosticStatus::OK;
            break;
          case ASN_COUNTER64:
          case ASN_INTEGER64:
            // kv.value = std::to_string(vars->val.counter64->high);
            // ds.values.push_back(kv);
            // ds.level = diagnostic_msgs::DiagnosticStatus::OK;
            break;
          case ASN_FLOAT:
            kv.value = std::to_string(*vars->val.floatVal);
            ds.values.push_back(kv);
            ds.level = diagnostic_msgs::DiagnosticStatus::OK;
            break;
          case ASN_DOUBLE:
            kv.value = std::to_string(*vars->val.doubleVal);
            ds.values.push_back(kv);
            ds.level = diagnostic_msgs::DiagnosticStatus::OK;
            break;
          default:
            kv.value = reinterpret_cast<char *>(vars->val.string);
            ds.values.push_back(kv);
            ds.level = diagnostic_msgs::DiagnosticStatus::OK;
            break;
        }
      }
    }
    if (response)
      snmp_free_pdu(response);


  }
  da.status.push_back(ds);
  diag_pub.publish(da);


}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "snmp_diag");

  host = ros::param::param<std::string>("~host", host);
  oids = ros::param::param("~oids", oids);
  std::cerr << oids.size() << " oids" << std::endl;

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
