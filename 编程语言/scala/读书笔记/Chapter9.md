# 读文件
```scala
import scala.io.Source
val source= Source.fromFile("path","UTF-8")
// todo:...
source.close
```

val lineIterator = source.getLines //读取行
val contents = source.mkString //读取整个文件
val iterchar = source // 读取char
val iter = source.buffered // source.buffered



# 其他地址
scala.io.Source.fromFile
scala.io.Source.fromURL
scala.io.Source.fromString

# 读取字节数组 

需要通过java的类库实现

# 文本写
没有写文本的类实现，需要通过java库

# 访问目录
通过java.nio.file._包中的Files.list和Files.walk

# 序列化类
实现java.io.Serializeble接口
class Person extends Serializable

scala中集合类都是可序列化的

# 和shell交互
scala.sys.process

# 正则表达
scala.util.matching.Regex
使用""" """构建字符串的Regex对象
调用findAllIn和findFirstIn方法
调用relaceFirstIn 、 relaceAllIn 、relaceSomeIn方法
调用findAllMatchIn 、 findFirstMatchIn 方法，返回match对象