# HIVE CLI

hive --help --service cli 输出以下帮助：

```
usage: hive
 -d,--define <key=value>          Variable substitution to apply to Hive
                                  commands. e.g. -d A=B or --define A=B
    --database <databasename>     Specify the database to use
 -e <quoted-query-string>         SQL from command line
 -f <filename>                    SQL from files
 -H,--help                        Print help information
    --hiveconf <property=value>   Use value for given property
    --hivevar <key=value>         Variable substitution to apply to Hive
                                  commands. e.g. --hivevar A=B
 -i <filename>                    Initialization SQL file
 -S,--silent                      Silent mode in interactive shell
 -v,--verbose                     Verbose mode (echo executed SQL to the
                                  console)

```
需要关注的配置包括：-e、-f

# HIVE中的变量

在HIVE SQL中能使用到的一些变量，包括：

- 通过--define、--hivevar定义的自定义变量
- 通过配置文件传递的HIVE、Hadoop相关配置
- JVM中以system打头的变量
- bash中以env打头的变量，这些变量不能修改，只能读取

通过set命令可以读写这些变量：

```
# 获取所有变量
set
# 获取单个变量
set mapreduce.framework.name;
set set env:HOME;
# 修改变量
set hive.exec.mode.local.auto=true;

```

# 使用.hiverc文件

通过$HOME/.hiverc文件，可以将一些初始化配置固化到文件中。如客户端参数的配置、添加JAR包依赖等等。通过-i参数，可以手工指定初始化文件的位置。

# 历史操作

CLI会将最近10000行命令记录到$HOME/.hivehistory中！

# 在CLI中执行命令


- 执行普通shell时，命令以“!”开头、分号结尾！
- 执行HDFS命令，命令通过"dfs"开头！
