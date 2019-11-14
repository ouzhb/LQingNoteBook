# 编译fat jar

在project目录，创建一个assembly.sbt文件，添加assembly插件的依赖，版本号要和build.properties中的sbt版本匹配（这里是1.2.4）

```scala
addSbtPlugin("com.eed3si9n" % "sbt-assembly" % "0.14.5")
```



编辑build.sbt

provided 表示不将这些jar编译到最终的jar包

assemblyMergeStrategy 是assembly插件的合并名class时的策略

```scala
name := "spark4jobserver"

version := "0.1"

scalaVersion := "2.11.12"

resolvers += "Job Server Bintray" at "https://dl.bintray.com/spark-jobserver/maven"

libraryDependencies ++= Seq(
  "spark.jobserver" %% "job-server-api" % "0.8.0" % "provided",
  "org.apache.spark" %% "spark-core" % "2.3.1" % "provided",
  "org.apache.spark" %% "spark-sql" % "2.3.1" % "provided",
  "org.apache.spark" %% "spark-streaming" % "2.3.1" % "provided",
  "org.mongodb.spark" %% "mongo-spark-connector" % "2.3.1"
)

assemblyMergeStrategy in assembly := {
  case x: String if x.contains("class") => MergeStrategy.first
  case x: String if x.contains("properties") => MergeStrategy.first
  case x: String if x.contains("jersey-module-version") => MergeStrategy.first
  case x =>
    val oldStrategy = (assemblyMergeStrategy in assembly).value
    oldStrategy(x)
}
```

最后使用assembly命令进行编译

# 参考
https://github.com/sbt/sbt-assembly